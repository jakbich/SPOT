#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os
import tf
import rospkg
import ros_numpy
import math
from sensor_msgs.msg import PointCloud2
from move_base_msgs.msg import MoveBaseGoal
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber,ApproximateTimeSynchronizer
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from yolo.msg import DetectionArray, DetectionInfo


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
        self.image_right = None
        self.image_left = None
        self.net = None
        self.intrinsic_calibration_right = None
        self.intrinsic_calibration_left = None
        self.detections = []
        rospy.init_node('detection')
        rospy.on_shutdown(self.shutdown)

        self.tf_listener = tf.TransformListener()
        rospy.sleep(rospy.Duration(2)) # Give the tf server some time to start up

        #define target and source frames of the cameras (TODO: get from param server)
        self.left_camera_source = '/frontleft' 
        self.right_camera_source = '/frontright'
        self.camera_target = '/base_link'

        # Get transformatin matrix between base footpint (origin of pointcloud) to each camera frame
        self.right_base_cam = self.get_transformation(str('/link'), str('/frontright'), rospy.Time.now())
        self.left_base_cam = self.get_transformation(str('/base_link'), str('/frontleft'), rospy.Time.now())

        #create publishers
        self.image_pub = rospy.Publisher('/spot/camera/boundingBoxCamera', Image, queue_size=2)
        self.pub_database = rospy.Publisher('/spot/database', DetectionArray, queue_size=10)
        
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
        sub_left_image = Subscriber('/spot/camera/frontleft/image', Image, queue_size=3)
        sub_right_image = Subscriber('/spot/camera/frontright/image', Image, queue_size=3)
        sub_robot_pos = Subscriber("/odom/ground_truth", Odometry, queue_size=15)
        sub_pcl_left_msg = Subscriber("/spot/depth/left/pointcloud", PointCloud2, queue_size=15)
        sub_pcl_right_msg = Subscriber("/spot/depth/right/pointcloud", PointCloud2, queue_size=15)
        self.pub_detection_markers = rospy.Publisher("/spot/detections/markers", MarkerArray, queue_size=3)

        # ts_left = ApproximateTimeSynchronizer([sub_left_image, sub_left_depth], queue_size=5, slop=0.1)
        # ts_left.registerCallback(self.callback_left)

        ts_right = ApproximateTimeSynchronizer([sub_left_image, sub_right_image, sub_robot_pos, sub_pcl_left_msg,sub_pcl_right_msg], queue_size=5, slop=0.5)
        ts_right.registerCallback(self.callback_right)

    def initialize_yolo(self):
        self.bridge = CvBridge()  # for conversion between OpenCV and ROS

        '''YOLO SETUP'''
        self.confidence_threshold = 0.45

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

        # Initialize colors
        self.classes = open(self.coco_names).read().strip().split('\n')
        np.random.seed(42)
        self.colors = np.random.randint(0, 255, size=(len(self.classes), 3), dtype='uint8')
        '''YOLO SETUP'''

    def get_camera_info(self):
        camera_info_msg_right = rospy.wait_for_message('/spot/camera/frontright/camera_info', CameraInfo)
        camera_info_msg_left = rospy.wait_for_message('/spot/camera/frontleft/camera_info', CameraInfo)
        self.intrinsic_calibration_right = camera_info_msg_right.K
        self.intrinsic_calibration_left = camera_info_msg_left.K


    def marker_array_msg(self):
        marker_array_msg = MarkerArray()
        i=0
        for detection in self.detections:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.type = Marker.SPHERE
            marker.id = i
            marker.action = Marker.ADD
            marker.pose.position.x = detection.position[0]
            marker.pose.position.y = detection.position[1]
            marker.pose.position.z = detection.position[2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            if detection.type == "person":
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0

            # Add the marker to the MarkerArray message
            marker_array_msg.markers.append(marker)

            i+=1
        self.pub_detection_markers.publish(marker_array_msg)



    def callback_right(self, left_image_msg, right_image_msg, robot_pos_msg,pcl_left_msg, pcl_right_msg) -> None:
        #rospy.loginfo("image_received")
        if self.net is not None:

            centers, confidences, class_names, areas, _ = self.detect(left_image_msg,"left")  #apply yolo to image
            #filter out detections that are to large
            if centers is not None:
                for i in range(len(centers)):
                    #extract information
                    center = centers[i]
                    class_name = class_names[i]
                    confidence = confidences[i]
                    area = areas[i]

                    #filter out detections with too large of an area
                    if area < 150000:
                        center3d = self.get_3d_point(center, pcl_left_msg, robot_pos_msg, self.right_base_cam)

                        #check if point already exist if so dont add this detection to the database
                        center3d = self.update_database(center3d,class_name,confidence)
                    else:
                        pass
            

            centers, confidences, class_names, areas, img_encoding = self.detect(right_image_msg,"right")
            if centers is not None:
                for i in range(len(centers)):
                    #extract information
                    center = centers[i]
                    class_name = class_names[i]
                    confidence = confidences[i]
                    area = areas[i]

                    #filter out detections with too large of an area
                    if area < 150000:
                        center3d = self.get_3d_point(center, pcl_right_msg, robot_pos_msg, self.right_base_cam)

                        #check if point already exist if so dont add this detection to the database
                        center3d = self.update_database(center3d,class_name,confidence)
                    else:
                        pass

            

            #stack both left and right images and publish them
            cv_image = np.hstack((self.image_right,self.image_left))
            bounding_box_image = self.bridge.cv2_to_imgmsg(cv_image, encoding=img_encoding)
            self.image_pub.publish(bounding_box_image)


            #publish the 3d points as a marker
            self.marker_array_msg()

            #publish the entire databse
            self.publish_database_msg()
                        
            # for detection in self.detections:
            #     rospy.loginfo(detection.__str__())

        else:
            rospy.logwarn("Image received but yolo not yet initialized")


    def publish_database_msg(self):
        rospy.loginfo("building database")

        i = 0
        detection_array_msg = DetectionArray()
        for detection in self.detections:

            header = Header()
            header.seq=i
            header.stamp = rospy.Time.now()
            header.frame_id = "odom"
            
            position = MoveBaseGoal()
            position.target_pose.header=header
            position.target_pose.pose.position.x = detection.position[0]
            position.target_pose.pose.position.y = detection.position[1]
            position.target_pose.pose.orientation.w = 1.0
            
            message = DetectionInfo()
            message.header = header
            message.position = position
            message.confidence = detection.confidence
            message.type = detection.type
            detection_array_msg.detections.append(message)

            i+=1

        self.pub_database.publish(detection_array_msg)
      

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
        areas = []
        if len(indices) > 0:
            for i in indices.flatten():
                if self.classes[classIDs[i]] == "apple" or self.classes[classIDs[i]] == "person":
                    
                    (x, y) = (boxes[i][0], boxes[i][1])
                    (w, h) = (boxes[i][2], boxes[i][3])
                    color = [int(c) for c in self.colors[classIDs[i]]]
                    cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                    text = "{}: {:.4f}".format(self.classes[classIDs[i]], confidences[i])
                    cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                    confidences_new.append(confidences[i])
                    classIDs_new.append(self.classes[classIDs[i]])
                    areas.append((x+w) * (y+h))

                    '''
                    image was rotated 90 degree before detection because otherwise yolo would struggle
                    This means that the found center point need to be rotated back
                    rotated_x = H - y_old -1
                    rotated_y = x_old
                    int(y+h/2) / int(x+w/2) give center point of bounding box
                    '''

                    centers.append([H - int(y+h/2)-1,int(x+w/2)])
        return centers,confidences_new,classIDs_new,areas
            
    def detect(self, img_msg,type):
        try:
            img_encoding = img_msg.encoding
            
            #convert msg into cv2 readable image
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, img_encoding)
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

            #if the image is gray, make it rgb like
            #cv_image_right = np.stack((cv_image_right,cv_image_right,cv_image_right),axis=-1)

            #yolov3 needs (416,416) input, yolov7 needs (640,640)
            blob = cv2.dnn.blobFromImage(cv_image, scalefactor=1/255, size=(640, 640), swapRB=False)
            self.net.setInput(blob)

            # determine the output layer
            layer_names = self.net.getLayerNames()
            ln = [layer_names[int(i - 1)] for i in self.net.getUnconnectedOutLayers()]

            outputs = np.vstack(self.net.forward(ln))

            #contains multiple post-prosessing steps including non-maximum suppression
            if type == "left":
                self.image_left = cv_image
                centers, confidences, class_names, areas = self.draw_bounding_box(outputs, self.image_left) 
            elif type == "right":
                self.image_right = cv_image
                centers, confidences, class_names, areas = self.draw_bounding_box(outputs, self.image_right) 
            
            return centers, confidences, class_names, areas,img_encoding
        except:
            return None, None, None, None, None
        
    def get_3d_point(self, center, pcl_msg, robot_pos_msg, trans_cam2base):
        # Convert from u (column / width), v (row/height) to position in array
        u, v = center[0], center[1]
        arrayPosition = v * pcl_msg.row_step + u * pcl_msg.point_step

        # Compute position in array where x, y, z data start
        arrayPosX = arrayPosition + pcl_msg.fields[0].offset  # X has an offset of 0
        arrayPosY = arrayPosition + pcl_msg.fields[1].offset  # Y has an offset of 4
        arrayPosZ = arrayPosition + pcl_msg.fields[2].offset  # Z has an offset of 8

        # Extract X, Y, Z values from the point cloud data
        x = np.frombuffer(pcl_msg.data, dtype=np.float32, count=1, offset=arrayPosX)[0]
        y = np.frombuffer(pcl_msg.data, dtype=np.float32, count=1, offset=arrayPosY)[0]
        z = np.frombuffer(pcl_msg.data, dtype=np.float32, count=1, offset=arrayPosZ)[0]
        point3d_cam = np.array([x, y, z]) # In camera frame

        # Transform to base_link frame
        point3d_cam_hom = np.hstack((point3d_cam, np.ones(1))) # Make pointcloud homogeneous
        point3d_base_hom = np.dot(trans_cam2base, point3d_cam_hom.T).T
        point3d_base = point3d_base_hom[:3] / point3d_base_hom[-1:]
        point3d_base = point3d_base[:2] # Discard z-coordinate

        # Get robot location
        robot_pose = robot_pos_msg.pose.pose
        robot_location = np.array([robot_pose.position.x, robot_pose.position.y, robot_pose.position.z])
        quaternions = np.array([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])

        # Get transformation from base_link frame to odom
        yaw = tf.transformations.euler_from_quaternion(quaternions, 'sxyz')[-1]
        trans_base2odom = np.array([[math.cos(yaw),  -math.sin(yaw),  robot_location[0]], 
                                    [math.sin(yaw),   math.cos(yaw),  robot_location[1]],
                                    [0,                           0,                  1]])
        
        # Transfrom to odom frame
        point3d_base_hom = np.hstack((point3d_base, np.ones(1)))
        point3d_odom_hom = np.dot(trans_base2odom, point3d_base_hom.T).T
        point3d_odom = point3d_odom_hom[:2] / point3d_odom_hom[-1:]

        return np.hstack((point3d_odom, np.zeros(1)))


    def get_transformation(self, source: str, target: str, time) -> np.ndarray:
        trans_matrix = np.zeros((4, 4))
        position, quaternions = None, None

        try:
            position, quaternions = self.tf_listener.lookupTransform(target, source, time)
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

    def update_database(self,position,class_name,confidence):
        if not self.detections:  
            detection = Detection(class_name, confidence, position)
            self.detections.append(detection) 
        
        # if a new point is closer than 30cm to any old point it is not added because it is assumed it is the same detection
        unique = True
        for detection in self.detections:
            distance = np.linalg.norm(detection.position[:2] - position[:2])
            if distance < 0.3: 
                unique = False
        if unique:
            detection = Detection(class_name, confidence, position)
            self.detections.append(detection) 
    
    def shutdown(self):
        pass

if __name__=='__main__':
    try:
        Yolo()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node 'yolo_detection' could not be launched")
        pass
