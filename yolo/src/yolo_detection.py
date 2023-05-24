#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os
import tf
import rospkg
import ros_numpy
from sensor_msgs.msg import PointCloud2
from geometry_msgs import PointStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from message_filters import Subscriber,ApproximateTimeSynchronizer
from cv_bridge import CvBridge, CvBridgeError



class Yolo:
    def __init__(self) -> None:
        self.cv_image = None
        self.net = None
        self.intrinsic_calibration_right = None
        self.intrinsic_calibration_left = None
        rospy.init_node('detection')
        rospy.on_shutdown(self.shutdown)
        self.loop_rate = rospy.Rate(1)

        self.tf_listener = tf.TransformListener()
        rospy.sleep(rospy.Duration(2)) # Give the tf server some time to start up

        #define target and source frames of the cameras (TODO: get from param server)
        self.left_camera_source = '/frontleft_rgb_optical_frame' 
        self.right_camera_source = '/frontright_rgb_optical_frame'
        self.camera_target = '/base_footprint'

        # Get transformatin matrix between base footpint (origin of pointcloud) to each camera frame
        self.right_trans = self.get_transformation(str('/base_footprint'),str('/frontright_rgb_optical_frame'))
        self.left_trans = self.get_transformation(str('/base_footprint'),str('/frontleft_rgb_optical_frame' ))

        #create publishers
        self.image_pub = rospy.Publisher('/spot/camera/boundingBoxCamera', Image, queue_size=2)
        self.points_pub = rospy.Publisher('/spot/depth/detection_center',PointStamped,queue_size=10)
    

        self.get_camera_info()

        if self.intrinsic_calibration_left == None:
            rospy.logfatal("Left Camera Calibration Matrix Not Obtained")
        
        elif self.intrinsic_calibration_right == None:
            rospy.logfatal("Right Camera Calibration Matrix Not Obtained")

        else:
            rospy.loginfo("Camera Information Successfully Obtained")
            self.intrinsic_calibration_right_inv = np.linalg.inv(np.array(self.intrinsic_calibration_right).reshape(3,3))
            self.intrinsic_calibration_left_inv = np.linalg.inv(np.array(self.intrinsic_calibration_left).reshape(3,3))


        self.initialize_yolo()
        rospy.loginfo("YOLO initialized")

        # Synchronize the subscribers based on their timestamps
        sub_left = Subscriber('/spot/camera/frontleft/image', Image, queue_size=2)
        sub_right = Subscriber('/spot/camera/frontright/image', Image, queue_size=2)
        sub_pc = Subscriber('/spot/depth/plane_segmentation/non_ground', PointCloud2, queue_size=2)

        ts = ApproximateTimeSynchronizer([sub_right, sub_left,sub_pc], queue_size=2, slop=0.5)
        ts.registerCallback(self.callback)

    def initialize_yolo(self):
        self.bridge = CvBridge()  # for conversion between OpenCV and ROS

        '''YOLO SETUP'''
        self.confidence_threshold = 0.1

        #get the file path to the yolo package
        rospack = rospkg.RosPack()
        path_relative = str(rospack.get_path('yolo'))

        # Construct the updated file path to yolo initialisation files
        file_path_cfg = 'yolo_config/yolov3.cfg'
        file_path_weight = 'yolo_config/yolov3.weights'
        file_path_names = 'yolo_config/coco.names'
        absolute_path_cfg = os.path.join(path_relative, file_path_cfg)
        absolute_path_weight = os.path.join(path_relative, file_path_weight)
        self.coco_names = os.path.join(path_relative, file_path_names)
        
        # load yolo model
        self.net = cv2.dnn.readNet(absolute_path_weight, absolute_path_cfg)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        '''YOLO SETUP'''

    def get_camera_info(self):
        camera_info_msg_right = rospy.wait_for_message('/spot/camera/frontright/camera_info', CameraInfo)
        camera_info_msg_left = rospy.wait_for_message('/spot/camera/frontleft/camera_info', CameraInfo)
        self.intrinsic_calibration_right = camera_info_msg_right.K
        self.intrinsic_calibration_left = camera_info_msg_left.K
 
    def shutdown(self):
        # Clean up code here (if any) before the node shuts down
        pass

    def run(self) -> None:
        while not rospy.is_shutdown():
            self.loop_rate.sleep()


    def callback(self,right_msg,left_msg,pc_msg) -> None:
        try:
            if self.net is not None:
                rospy.loginfo('Data received...')
                img_encoding = left_msg.encoding
                
                #convert msg into cv2 readable image
                cv_image_left = self.bridge.imgmsg_to_cv2(left_msg,img_encoding)
                cv_image_right = self.bridge.imgmsg_to_cv2(right_msg,img_encoding)

                cv_image_left = cv2.rotate(cv_image_left, cv2.ROTATE_90_COUNTERCLOCKWISE)
                cv_image_right = cv2.rotate(cv_image_right, cv2.ROTATE_90_COUNTERCLOCKWISE)

                #self.cv_image = np.concatenate((cv_image_right, cv_image_left), axis=1)

                self.cv_image = cv_image_right

                outputs = self.detect(self.cv_image)      #apply yolo to image
                centers = self.draw_bounding_box(outputs,self.cv_image)  #contains multiple post-prosessing steps including non-maximum suppression

                # turn pc_msg to numpy array
                pc_base = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg)
                pc_hom_base = np.hstack((pc_base, np.ones(len(pc_base)).reshape(-1, 1)))

                # Apply transformation and convert back to normal coordinates
                pc_hom_camR = np.dot(self.right_trans, pc_hom_base.T).T
                pc_camR = pc_hom_camR[:, :3] / pc_hom_camR[:, -1:]

                #for each detection by yolo find the closest pointcloud point that corresponds to this pixel coordinate
                center_3d_base = []
                for center in centers:
                    pixel_hom = np.array([center[0],center[1],1]).reshape(3,1)
                    pixels = self.intrinsic_calibration_right_inv.dot(pixel_hom)
                    x_c, y_c, _ = pixels.flatten()

                    # Find the closest point in the point cloud
                    distances = np.sqrt(np.sum((pc_camR[:, :2] - np.array([x_c, y_c])) ** 2, axis=1))
                    closest_index = np.argmin(distances)
                    closest_point_cam = (pc_camR[closest_index])

                    #transform point back to base frame and append it to the list
                    closest_point_base_hom = np.dot(np.linalg.inv(self.right_trans),np.append(closest_point_cam,1))
                    center_3d_base.append(closest_point_base_hom[:3]/closest_point_base_hom[3])

                #contruct center point message
                point_msg = PointStamped()
                point_msg.header.stamp = rospy.Time.now()
                point_msg.header.frame_id = "base_footprint"
                point_msg.points = center_3d_base

                # Convert the image to ROS format and publish it
                bounding_box_image = self.bridge.cv2_to_imgmsg(self.cv_image, encoding=img_encoding)
                self.image_pub.publish(bounding_box_image)
                self.points_pub.publish(point_msg)

                rospy.loginfo('Information published')
            else:
                rospy.logwarn("Image received but yolo not yet initialized")

        except CvBridgeError as e:
            rospy.logerr(e)


    def draw_bounding_box(self,outputs,image):
        H,W = image.shape[:2]

        boxes = []
        confidences = []
        classIDs = []

        for output in outputs:
            scores = output[5:]
            classID = np.argmax(scores)
            confidence = scores[classID]
            if confidence > self.confidence_threshold:
                x, y, w, h = output[:4] * np.array([W, H, W, H])
                p0 = int(x - w//2), int(y - h//2)
                p1 = int(x + w//2), int(y + h//2)
                boxes.append([*p0, int(w), int(h)])
                confidences.append(float(confidence))
                classIDs.append(classID)

        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold, self.confidence_threshold-0.1)
        centers = []
        if len(indices) > 0:
            for i in indices.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
                color = [int(c) for c in self.colors[classIDs[i]]]
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(self.classes[classIDs[i]], confidences[i])
                cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

                '''
                image was rotated 90 degree before detection because otherwise yolo would struggle
                This means that the found center point need to be rotated back
                rotated_x = H - y_old -1
                rotated_y = x_old
                int(y+h/2) / int(x+w/2) give center point of bounding box
                '''

                centers.append([H - int(y+h/2)-1,int(x+w/2)])
        return centers
            
    def detect(self,img):
        self.classes = open(self.coco_names).read().strip().split('\n')
        np.random.seed(42)
        self.colors = np.random.randint(0, 255, size=(len(self.classes), 3), dtype='uint8')

        blob = cv2.dnn.blobFromImage(img, 1/255.0, (416, 416), swapRB=False, crop=False)
        self.net.setInput(blob)

        # determine the output layer
        layer_names = self.net.getLayerNames()
        ln = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
        
        return np.vstack(self.net.forward(ln))

    def get_transformation(self, source: str, target: str) -> np.ndarray:
        trans_matrix = np.zeros((4, 4))
        position, quaternions = None, None

        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(target, source, now, rospy.Duration(2.0))
            position, quaternions = self.tf_listener.lookupTransform(target, source, rospy.Time.now())
        except tf.Exception as e:
            rospy.logwarn("Failed to lookup transform: {}".format(e))
            
        if position is not None and quaternions is not None:
            euler = tf.transformations.euler_from_quaternion(quaternions, 'sxyz')
            trans_matrix = tf.transformations.euler_matrix(euler[0], euler[1], euler[2], axes='sxyz')

            trans_matrix[:3, 3] = np.array(position).T
            trans_matrix[3, 3] = 1

            return trans_matrix
        else:
            rospy.logfatal("Did not return")


if __name__=='__main__':
    try:
        node = Yolo()
        node.run()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node 'yolo_detection' could not be launched")
        pass