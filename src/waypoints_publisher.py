#!/usr/bin/env python3

import rospy
import yaml
from geometry_msgs.msg import Polygon, Point32

def load_waypoints_from_yaml(file_path):
    # Load waypoints from the YAML file
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
        return data['waypoints']  # Return the list of waypoints

def publish_waypoints():
    # Initialize the ROS node
    rospy.init_node('waypoint_publisher', anonymous=True)

    # Create a publisher for the waypoints topic
    waypoint_pub = rospy.Publisher('waypoints', Polygon, queue_size=10)

    # Load the waypoints from the YAML file
    waypoints = load_waypoints_from_yaml('/workspace/assignment/src/position_control/waypoints.yaml')

    # Convert the waypoints into a Polygon message
    polygon_msg = Polygon()
    for wp in waypoints:
        point = Point32()
        point.x, point.y, point.z = wp
        polygon_msg.points.append(point)

    rate = rospy.Rate(1)  # Publish at 1 Hz

    rospy.loginfo("Publishing waypoints...")

    while not rospy.is_shutdown():
         # Publish the waypoints
        waypoint_pub.publish(polygon_msg)
        rospy.loginfo("Waypoints published: %s", polygon_msg)
    
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_waypoints()
    except rospy.ROSInterruptException:
        pass
