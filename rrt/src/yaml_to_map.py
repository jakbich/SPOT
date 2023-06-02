#!/usr/bin/env python

import os
import rospy
import yaml
from nav_msgs.msg import OccupancyGrid

def yaml_to_map():
    rospy.init_node('yaml_to_map_node')

    # Get the path to the current file
    current_file_path = os.path.abspath(__file__)

    # Construct the relative path to the YAML file
    file_path = os.path.join(os.path.dirname(os.path.dirname(current_file_path)),  'map_final.yaml')

    # Open the YAML file
    with open(file_path, 'r') as file:
        map_data = yaml.safe_load(file)

    # Extract map information from the YAML datas
    data = map_data['data']
    width = map_data['width']
    height = map_data['height']
    resolution = map_data['resolution']
    origin_x = map_data['origin']['position']['x']
    origin_y = map_data['origin']['position']['y']

    # Create the OccupancyGrid message
    map_msg = OccupancyGrid()
    map_msg.header.frame_id = 'odom'
    map_msg.info.width = width
    map_msg.info.height = height
    map_msg.info.resolution = resolution
    map_msg.info.origin.position.x = origin_x
    map_msg.info.origin.position.y = origin_y
    map_msg.info.origin.orientation.w = 1.0
    map_msg.data = data

    # Publish the map on the /map topic
    map_pub = rospy.Publisher('/spot/mapping/map', OccupancyGrid, latch=True, queue_size=1)
    rospy.sleep(1)  # Wait for the publisher to connect
    map_pub.publish(map_msg)

    rospy.loginfo('Map published on /map topic')

if __name__ == '__main__':
    try:
        yaml_to_map()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass