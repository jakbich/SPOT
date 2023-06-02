#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os
import tf
import rospkg
import ros_numpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber,ApproximateTimeSynchronizer
from cv_bridge import CvBridge, CvBridgeError


class Detection:
    def __init__(self,type,confidence,position):
        self.type = type
        self.confidence = confidence
        self.position = position

    #stringyfies the data for printing
    def __str__(self):
        return f"Type: {self.type}, Confidence: {self.confidence}, Location: {self.position}"



class Yolo():
    def __init__(self) -> None:
        self.cv_image = None
        self.net = None
        self.intrinsic_calibration_right = None
        self.intrinsic_calibration_left = None
        self.detections = []
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
        self.right_base_cam = self.get_transformation(str('/base_footprint'),str('/frontright_rgb_optical_frame'))
        self.left_base_cam = self.get_transformation(str('/base_footprint'),str('/frontleft_rgb_optical_frame' ))


        

        #create publishers
        self.image_pub = rospy.Publisher('/spot/camera/boundingBoxCamera', Image, queue_size=2)
        
        self.get_camera_info()

        if self.intrinsic_calibration_left == None:
            rospy.logfatal("Left Camera Calibration Matrix Not Obtained")
        
        elif self.intrinsic_calibration_right == None:
            rospy.logfatal("Right Camera Calibration Matrix Not Obtained")

        else:
            rospy.loginfo("Camera Information Successfully Obtained")
            self.intrinsic_calibration_right = np.array(self.intrinsic_calibration_right).reshape(3,3)
            self.intrinsic_calibration_left = np.array(self.intrinsic_calibration_left).reshape(3,3)


        self.initialize_yolo()
        rospy.loginfo("YOLO initialized")

        # Synchronize the subscribers based on their timestamps
        sub_left = Subscriber('/spot/camera/frontleft/image', Image, queue_size=3)
        sub_right = Subscriber('/spot/camera/frontright/image', Image, queue_size=3)
        sub_pc = Subscriber('/spot/depth/plane_segmentation/non_ground', PointCloud2, queue_size=3)

        #sub_right = Subscriber('/test', Image, queue_size=2)

        ts = ApproximateTimeSynchronizer([sub_right, sub_left,sub_pc], queue_size=3, slop=1)
        # ts = ApproximateTimeSynchronizer([sub_right], queue_size=2, slop=0.5)
        ts.registerCallback(self.callback)

    def initialize_yolo(self):
        self.bridge = CvBridge()  # for conversion between OpenCV and ROS

        '''YOLO SETUP'''
        self.confidence_threshold = 0.45

        #get the file path to the yolo package
        rospack = rospkg.RosPack()
        path_relative = str(rospack.get_path('yolo'))

        # Construct the updated file path to yolo initialisation files
        file_path_cfg = 'yolo_config/yolov7x.cfg'
        file_path_weight = 'yolo_config/yolov7x.weights'
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
                img_encoding = right_msg.encoding
                
                #convert msg into cv2 readable image
                cv_image_left = self.bridge.imgmsg_to_cv2(left_msg,img_encoding)
                cv_image_right = self.bridge.imgmsg_to_cv2(right_msg,img_encoding)

                cv_image_left = cv2.rotate(cv_image_left, cv2.ROTATE_90_COUNTERCLOCKWISE)
                cv_image_right = cv2.rotate(cv_image_right, cv2.ROTATE_90_COUNTERCLOCKWISE)

                #if the image is gray, make it rgb like
                #cv_image_right = np.stack((cv_image_right,cv_image_right,cv_image_right),axis=-1)
              
                self.cv_image = cv_image_right

                outputs = self.detect(self.cv_image)      #apply yolo to image
                centers,confidences,class_names = self.draw_bounding_box(outputs,self.cv_image)  #contains multiple post-prosessing steps including non-maximum suppression
                rospy.loginfo(self.intrinsic_calibration_right)

                #turn pc_msg to numpy array
                pc_base = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg)
                pc_hom_base = np.hstack((pc_base, np.ones(len(pc_base)).reshape(-1, 1)))

                # Apply transformation and convert back to normal coordinates
                pc_hom_camR = np.dot(self.right_base_cam, pc_hom_base.T).T
                pc_camR = pc_hom_camR[:, :3] / pc_hom_camR[:, -1:]

                #for each detection by yolo find the closest pointcloud point that corresponds to this pixel coordinate
                #TODO: Figure out if the point found is accurate to the detection
               
                for i in range(len(centers)):
                    #extract information
                    center = centers[i]
                    class_name = class_names[i]
                    confidence = confidences[i]
                    
               

                    #transform pc points to pixel coordinates in camera frame, then check closest point
                    uvs = (self.intrinsic_calibration_right @ pc_camR.T).T
                    uv = uvs[:,:2] / uvs[0,-1]
                    dist = np.linalg.norm(uv-center,axis=1)
                    index = np.argmin(dist)

                    closest_point = uvs[index,:]
                    closest_point_camR = closest_point.dot(np.linalg.inv(self.intrinsic_calibration_right))

                    right_cam_odom = self.get_transformation(str('frontright_rgb_optical_frame'),str('odom'))
                    left_cam_odom = self.get_transformation(str('/frontleft_rgb_optical_frame' ),str('/odom'))

                    #transform point to odom frame 
                    closest_point_odom_hom = np.dot(right_cam_odom,np.append(closest_point_camR,1))
                    closest_point_odom =closest_point_odom_hom[:3]/closest_point_odom_hom[3]

                    #check if point already exist if so dont add this detection to the database
                    unique = self.is_unique(closest_point_odom)

                    if unique:
                        detection = Detection(class_name,confidence,closest_point_odom)
                        self.detections.append(detection)

                for detection in self.detections:
                    rospy.loginfo(detection.__str__())
               
                # Convert the image to ROS format and publish it
                #bounding_box_image = self.bridge.cv2_to_imgmsg(self.cv_image[:,:,0], encoding=img_encoding)
                bounding_box_image = self.bridge.cv2_to_imgmsg(self.cv_image, encoding=img_encoding)
                self.image_pub.publish(bounding_box_image)

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
        confidences_new = []
        classIDs_new = []
        if len(indices) > 0:
            for i in indices.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
                color = [int(c) for c in self.colors[classIDs[i]]]
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(self.classes[classIDs[i]], confidences[i])
                cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                confidences_new.append(confidences[i])
                classIDs_new.append(self.classes[classIDs[i]])

                '''
                image was rotated 90 degree before detection because otherwise yolo would struggle
                This means that the found center point need to be rotated back
                rotated_x = H - y_old -1
                rotated_y = x_old
                int(y+h/2) / int(x+w/2) give center point of bounding box
                '''

                centers.append([H - int(y+h/2)-1,int(x+w/2)])
        return centers,confidences_new,classIDs_new
            
    def detect(self,img):
        self.classes = open(self.coco_names).read().strip().split('\n')
        np.random.seed(42)
        self.colors = np.random.randint(0, 255, size=(len(self.classes), 3), dtype='uint8')

        #yolov3 needs (416,416) input, yolov7 needs (640,640)
        blob = cv2.dnn.blobFromImage(img, scalefactor=1/255, size=(640, 640),swapRB=False)
        self.net.setInput(blob)

        # determine the output layer
        layer_names = self.net.getLayerNames()
        ln = [layer_names[int(i - 1)] for i in self.net.getUnconnectedOutLayers()]
        
        return np.vstack(self.net.forward(ln))

    def get_transformation(self, source: str, target: str) -> np.ndarray:
        trans_matrix = np.zeros((4, 4))
        position, quaternions = None, None

        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(target, source, now, rospy.Duration(2.0))
            position, quaternions = self.tf_listener.lookupTransform(target, source, now)
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

    def is_unique(self, new_detection):
        if not self.detections:  
            return True 
        
        # if a new point is closer than 30cm to any old point it is not added because it is assumed it is the same detection
        for detection in self.detections:
            distance = np.linalg.norm(detection.position[:2] - new_detection[:2])
            if distance < 0.3: 
                return False
        return True


if __name__=='__main__':
    try:
        node = Yolo()
        node.run()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node 'yolo_detection' could not be launched")
        pass
