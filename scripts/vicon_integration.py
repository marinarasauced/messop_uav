#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
import json


class StateUAV:
    def __init__(self):
        # Update the coordinates of the UAV for the main code
        self.Px = None
        self.Py = None
        self.Pz = None

        self.vicon_offsetX = None
        self.vicon_offsetY = None
        self.vicon_offsetZ = None
        self.vicon_offsetQ = None
        self.offsets_set = False

        self.pub = rospy.Publisher(
            "/mavros/vision_pose/pose", PoseStamped, queue_size=10
        )

    def save_local_position(self, local_state):
        while not rospy.is_shutdown():
            self.Px = local_state.pose.position.x
            self.Py = local_state.pose.position.y
            self.Pz = local_state.pose.position.z

    def vicon_subscriber(self):
        rospy.init_node("pose_update", anonymous=True)
        rospy.Subscriber(
            "/vicon/hawk2/hawk2", TransformStamped, self.publish_vicon_position
        )
        rospy.spn()

    def publish_vicon_position(self, vicon_data):
        # rospy.init_node('local_position_publisher', anonymous=True)

        rate = rospy.Rate(
            10
        )  # Use this to dictate the frequency that location is published

        if not self.offsets_set:
            self.set_vicon_origin(vicon_position=vicon_data)
            self.offsets_set = True

        # Constantly update the location of the UAV
        while not rospy.is_shutdown():
            # Create a PoseStamped message
            local_pose = PoseStamped()
            local_pose.header.stamp = rospy.Time.now()
            local_pose.header.frame_id = "map"  # Indicate that coordinates are global

            # Set local position
            local_pose.pose.position.x = (
                vicon_data.tranform.translation.x - self.vicon_offsetX
            )
            local_pose.pose.position.y = (
                vicon_data.tranform.translation.y - self.vicon_offsetY
            )
            local_pose.pose.position.z = (
                vicon_data.tranform.translation.z - self.vicon_offsetZ
            )

            # Set local orientation
            Q_vicon = Quaternion()
            Q_vicon.x = vicon_data.transform.rotation.x
            Q_vicon.y = vicon_data.transform.rotation.y
            Q_vicon.z = vicon_data.transform.rotation.z
            Q_vicon.w = vicon_data.transform.rotation.w
            Q_real = self.quat_multiply(Q_vicon)

            local_pose.pose.orientation.x = Q_real.x
            local_pose.pose.orientation.y = Q_real.y
            local_pose.pose.orientation.z = Q_real.z
            local_pose.pose.orientation.w = Q_real.w

            # Publish to mavros
            self.pub.publish(local_pose)

            rate.sleep()

    def set_vicon_origin(self, vicon_position):
        f = open("data.json")
        data = json.load(f)
        Q1 = Quaternion()
        Q1.x = data["x"]
        Q1.y = data["y"]
        Q1.z = data["z"]
        Q1.w = data["w"]
        self.vicon_offsetQ = Q1
        self.vicon_offsetX = -vicon_position.transform.translation.x
        self.vicon_offsetY = -vicon_position.transform.translation.y
        self.vicon_offsetZ = -vicon_position.transform.translation.z

    def quat_multiply(self, quat):
        # Product of Quaternion input:
        qnew = Quaternion()
        qnew.x = (
            (self.vicon_offsetQ.w * quat.x)
            + (self.vicon_offsetQ.x * quat.w)
            + (self.vicon_offsetQ.y * quat.z)
            - (self.vicon_offsetQ.z * quat.y)
        )
        qnew.y = (
            (self.vicon_offsetQ.w * quat.y)
            - (self.vicon_offsetQ.x * quat.z)
            + (self.vicon_offsetQ.y * quat.w)
            + (self.vicon_offsetQ.z * quat.x)
        )
        qnew.z = (
            (self.vicon_offsetQ.w * quat.z)
            + (self.vicon_offsetQ.x * quat.y)
            - (self.vicon_offsetQ.y * quat.x)
            + (self.vicon_offsetQ.z * quat.w)
        )
        qnew.w = (
            (self.vicon_offsetQ.w * quat.w)
            - (self.vicon_offsetQ.x * quat.x)
            - (self.vicon_offsetQ.y * quat.y)
            - (self.vicon_offsetQ.z * quat.z)
        )

        return qnew
