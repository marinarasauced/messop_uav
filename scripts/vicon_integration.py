#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import ParamSet


class StateUAV:
    def __init__(self):
        # The codes knowledge of where the UAV is
        self.Px = None
        self.Py = None
        self.Pz = None
        self.error_tol = 0.1    # in meters
        self.pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=10)

    def publish_local_position(self, vicon_data):
        # rospy.init_node('local_position_publisher', anonymous=True)

        rate = rospy.Rate(10)  # Use this to dictate the frequency that location is published

        # Constantly update the location of the UAV
        while not rospy.is_shutdown():
            # Create a PoseStamped message
            local_pose = PoseStamped()
            local_pose.header.stamp = rospy.Time.now()
            local_pose.header.frame_id = 'map'  # Indicate that coordinates are global

            # Set local position
            local_pose.pose.position.x = 1.0
            local_pose.pose.position.y = 2.0
            local_pose.pose.position.z = 3.0

            # Set local orientation
            local_pose.pose.orientation.x = 1.0
            local_pose.pose.orientation.y = 1.0
            local_pose.pose.orientation.z = 1.0
            local_pose.pose.orientation.w = 1.0

            # Publish to mavros
            self.pub.publish(local_pose)

            rate.sleep()

    def center_vicon_coordinates(self):

