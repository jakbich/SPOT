#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from message_filters import TimeSynchronizer, Subscriber,ApproximateTimeSynchronizer
from cv_bridge import CvBridge, CvBridgeError
import tf
import rospkg


class Yolo:
    def __init__(self) -> None:
        self.cv_image = None
        self.net = None
        rospy.init_node('detection')
        rospy.on_shutdown(self.shutdown)
        self.loop_rate = rospy.Rate(1)

        self.tf_listener = tf.TransformListener()
        rospy.sleep(rospy.Duration(2)) # Give the tf server some time to start up

        #define target and source frames of the camera
        right_camera_source = '/frontright_rgb_optical_frame'
        left_camera_source = '/frontleft_rgb_optical_frame'
        camera_target = '/front_rail'

        #Get fixed transformation matrix between cameras and base of robot
        #self.right_trans = self.get_transformation(right_camera_source,camera_target)
        #self.left_trans = self.get_transformation(left_camera_source,camera_target)
        
        self.image_pub = rospy.Publisher('/spot/camera/boundingBoxCamera', Image, queue_size=2)
     
        self.initialize_yolo()

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

        # Synchronize the subscribers based on their timestamps
        sub_left = Subscriber('/spot/camera/frontleft/image', Image, queue_size=2)
        sub_right = Subscriber('/spot/camera/frontright/image', Image, queue_size=2)
        ts = ApproximateTimeSynchronizer([sub_right, sub_left],queue_size=2,slop=0.1)
        ts.registerCallback(self.callback)

        rospy.loginfo("YOLO initialised")

    
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
                
                #convert msg into cv2 readable image
                cv_image_left = self.bridge.imgmsg_to_cv2(left_msg,img_encoding)
                cv_image_right = self.bridge.imgmsg_to_cv2(right_msg,img_encoding)
               
                #self.test_transform = cv2.warpPerspective(cv_image, self.left_trans[:3,:3], (cv_image.shape[1], cv_image.shape[0]))
                
                cv_image_left = cv2.rotate(cv_image_left, cv2.ROTATE_90_COUNTERCLOCKWISE)
                cv_image_right = cv2.rotate(cv_image_right, cv2.ROTATE_90_COUNTERCLOCKWISE)

                self.cv_image = np.concatenate((cv_image_right, cv_image_left), axis=1)

                outputs = self.detect(self.cv_image)      #apply yolo to image
                self.draw_bounding_box(outputs,self.cv_image)  #contains multiple post-prosessing steps including non-maximum suppression

                # Convert the image to ROS format and publish it
                bounding_box_image = self.bridge.cv2_to_imgmsg(self.cv_image , encoding=img_encoding)
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
        
        #returns outputs
        return np.vstack(self.net.forward(ln))

if __name__=='__main__':
    try:
        node = Yolo()
        node.run()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node 'yolo_detection' could not be launched")
        pass