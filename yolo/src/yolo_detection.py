#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from message_filters import TimeSynchronizer, Subscriber,ApproximateTimeSynchronizer
from cv_bridge import CvBridge, CvBridgeError
import tf
import rospkg


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

        #for publishing the image with detections
        self.image_pub = rospy.Publisher('/spot/camera/boundingBoxCamera', Image, queue_size=2)
    

        self.get_camera_info()

        if self.intrinsic_calibration_left == None:
            rospy.logfatal("Left Camera Calibration Matrix Not Obtained")
        
        elif self.intrinsic_calibration_right == None:
            rospy.logfatal("Right Camera Calibration Matrix Not Obtained")

        else:
            rospy.loginfo("Camera Information Successufully Obtained")
            self.intrinsic_calibration_right = np.array(self.intrinsic_calibration_right).reshape(3,3)
            self.intrinsic_calibration_left = np.array(self.intrinsic_calibration_left).reshape(3,3)


        self.initialize_yolo()
        rospy.loginfo("YOLO initialized")

        # Synchronize the subscribers based on their timestamps
        sub_left = Subscriber('/spot/camera/frontleft/image', Image, queue_size=2)
        sub_right = Subscriber('/spot/camera/frontright/image', Image, queue_size=2)

        ts = ApproximateTimeSynchronizer([sub_right, sub_left], queue_size=2, slop=0.1)
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


    def callback(self,right_msg,left_msg) -> None:
        try:
            if self.net is not None:
                rospy.loginfo('Image\'s received...')
                img_encoding = left_msg.encoding
                rospy.loginfo(img_encoding)
                
                #convert msg into cv2 readable image
                cv_image_left = self.bridge.imgmsg_to_cv2(left_msg,img_encoding)
                cv_image_right = self.bridge.imgmsg_to_cv2(right_msg,img_encoding)

                self.cv_image = self.transform_image(cv_image_right,self.intrinsic_calibration_right)

                #cv_image_left = cv2.rotate(cv_image_left, cv2.ROTATE_90_COUNTERCLOCKWISE)
                #cv_image_right = cv2.rotate(cv_image_right, cv2.ROTATE_90_COUNTERCLOCKWISE)

                #self.cv_image = np.concatenate((cv_image_right, cv_image_left), axis=1)

                #outputs = self.detect(self.cv_image)      #apply yolo to image
                #self.draw_bounding_box(outputs,self.cv_image)  #contains multiple post-prosessing steps including non-maximum suppression

                # Convert the image to ROS format and publish it
                bounding_box_image = self.bridge.cv2_to_imgmsg(cv_image_right, encoding=img_encoding)
                self.image_pub.publish(bounding_box_image)

                rospy.loginfo('Image published')
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
        if len(indices) > 0:
            for i in indices.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
                color = [int(c) for c in self.colors[classIDs[i]]]
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(self.classes[classIDs[i]], confidences[i])
                cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
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

    # def transform_image(self, image_A, frame_A, frame_B, K):
    #     try:
    #         now = rospy.Time.now()
    #         self.tf_listener.waitForTransform(frame_A, frame_B, now, rospy.Duration(2.0))
    #         position, quaternions = self.tf_listener.lookupTransform(frame_A, frame_B, now)
    #     except (tf.Exception, tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException) as e:
    #         rospy.logwarn("Failed to lookup transform: {}".format(e))
    #         return None

    #     if position is not None and quaternions is not None:
    #         euler = tf.transformations.euler_from_quaternion(quaternions, axes='sxyz')
    #         trans_matrix = tf.transformations.compose_matrix(translate=position, angles=euler)
            
    #         height, width = image_A.shape[:2]
    #         image_B = np.zeros_like(image_A)

    #         K_inv = np.linalg.inv(K)  # Inverse of camera intrinsic calibration matrix

    #         for y in range(height):
    #             for x in range(width):
    #                 # Convert pixel coordinates to normalized image coordinates
    #                 pixel_homogeneous = np.array([x, y, 1])  # Homogeneous coordinates
    #                 pixel_normalized = np.dot(K_inv, pixel_homogeneous)
    #                 pixel_normalized /= pixel_normalized[2]  # Divide by last element

    #                 # Convert normalized image coordinates to 3D camera coordinates
    #                 u, v = pixel_normalized[:2]  # Extract x, y coordinates

    #                 # Apply transformation matrix to the 3D point
    #                 point_A_homogeneous = np.array([u, v, 1, 1])  # Homogeneous coordinates
    #                 point_B_homogeneous = np.dot(trans_matrix, point_A_homogeneous)
    #                 point_B = point_B_homogeneous[:3] / point_B_homogeneous[3]

    #                 # Project transformed 3D point onto ground plane
    #                 u_new, v_new, _ = point_B  # Ignore the z-coordinate

    #                 # Set pixel value in transformed image
    #                 if 0 <= u_new < width and 0 <= v_new < height:
    #                     image_B[int(v_new), int(u_new)] = image_A[y, x]

    #         return image_B
    #     else:
    #         rospy.logfatal("Failed to obtain transformation")
    #         return None
        


    def transform_image(self, image_A, K):
        height, width, channels = image_A.shape[:2]
        image_B = np.zeros_like(image_A)

        K_inv = np.linalg.inv(K)  # Inverse of camera intrinsic calibration matrix

        for y in range(height):
            for x in range(width):
                # Convert pixel coordinates to normalized image coordinates
                pixel_homogeneous = np.array([x, y, 1])  # Homogeneous coordinates
                pixel_normalized = np.dot(K_inv, pixel_homogeneous)
                pixel_normalized /= pixel_normalized[2]  # Divide by last element

                # Apply transformation to the normalized coordinates
                u, v = pixel_normalized[:2]  # Extract x, y coordinates

                # Set transformed pixel coordinates in new image
                u_new, v_new = u, v  # No transformation applied for now

                # Set pixel value in transformed image
                if 0 <= u_new < width and 0 <= v_new < height:
                    for c in range(channels):
                        image_B[int(v_new), int(u_new), c] = image_A[y, x, c]

        return image_B



      



       

if __name__=='__main__':
    try:
        node = Yolo()
        node.run()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node 'yolo_detection' could not be launched")
        pass