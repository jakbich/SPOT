#!/usr/bin/env python

import rosbag
from std_msgs.msg import Header

def change_frame_id(input, output, new_frame):
    with rosbag.Bag(output, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input).read_messages():
            msg.header.frame_id = new_frame
            outbag.write = (topic, msg, t)

if __name__ == "__main__":
    input = "/home/guido/mdp_spot/depth_image.bag"
    output = "/home/guido/mdp_spot/output.bag"
    new_frame = "odom"

    change_frame_id(input, output, new_frame)