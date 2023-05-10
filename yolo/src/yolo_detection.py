#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from message_filters import TimeSynchronizer, Subscriber
from cv_bridge import CvBridge, CvBridgeError

class Yolo:
    def __init__(self) -> None:
        rospy.init_node('detection')
        rospy.on_shutdown(self.shutdown)
        self.initialize_yolo()


    def initialize_yolo(self):
        self.rate = rospy.Rate(4)
        self.bridge = CvBridge()  # for conversion between OpenCV and ROS

        '''YOLO SETUP'''
        self.confidence_threshold = 0.5

        # Get the user's home directory
        home_dir = os.path.expanduser("~")

        # Construct the updated file path
        file_path_cfg = 'mdp_spot/src/champ_spot/yolo/yolo_config/cfg/yolov3.cfg'
        file_path_weight = 'mdp_spot/src/champ_spot/yolo/yolo_config/yolov3.weights'
        relative_path_cfg = os.path.join(home_dir, file_path_cfg)
        relative_path_weight = os.path.join(home_dir,file_path_weight)

        # load yolo model
        self.net = cv2.dnn.readNet(relative_path_weight, relative_path_cfg) 
        '''YOLO SETUP'''

        self.image_sub = rospy.Subscriber('/spot/camera/frontleft/image', Image, self.callback)
        self.box_pub = rospy.Publisher('/spot/2Dboundingbox', BoundingBox2D, queue_size=2)

    
    def run(self) -> None:
        while not rospy.is_shutdown():
            self.rate.sleep()

    def callback(self,msg) -> None:
        try:
            #convert msg into cv2 readable image
            cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            height,width,channel = cv_image.shape

            #apply the net to find detections
            outputs = self.preprocess_image(cv_image)

            #extract all predictions
            class_ids = []
            confidences = []
            boxes = []

            for output in outputs:
                for detection in output:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > self.confidence_threshold:
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)
                        x = int(center_x - w/2)
                        y = int(center_y - h/2)

                        boxes.append([x,y,w,h])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)

            indexes =  cv2.dnn.NMSBoxes(boxes,confidences,0.5,0.4)
            for i in indexes.flatten():
                box = BoundingBox2D()
                box.header = msg.header
                box.center = Point()
                box.center.x = (boxes[i][0] + boxes[i][2]) / 2.0
                box.center.y = (boxes[i][1] + boxes[i][3]) / 2.0
                box.size_x = boxes[i][2]
                box.size_y = boxes[i][3]
                box.probability = confidences[i]

                self.box_pub.publish(box)

            
            # Draw bounding boxes on the image
            # TODO: Add your code here for drawing bounding boxes on the image


    

        except CvBridgeError as e:
            rospy.logerr(e)


    def preprocess_image(self, cv_image):
        blob = cv2.dnn.blobFromImage(cv_image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        layer_names = self.net.getLayerNames()
        output_layers = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
        outputs = self.net.forward(output_layers)
        return outputs
    
    def shutdown(self):
        # Clean up code here (if any) before the node shuts down
        pass
    
if __name__=='__main__':
    try:
        node = Yolo()
        node.run()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node 'yolo_detection' could not be launched")
        pass