import rospy
from geometry_msgs.msg import PoseStamped


def publish_local_position():
    pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=10)
    rospy.init_node('local_position_publisher', anonymous=True)

    rate = rospy.Rate(10)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        # Create a PoseStamped message
        local_pose = PoseStamped()
        local_pose.header.stamp = rospy.Time.now()
        local_pose.header.frame_id = 'map'  # Indicate that coordinates are global

        # Set local position
        local_pose.pose.position.x = 1.0
        local_pose.pose.position.y = 2.0
        local_pose.pose.position.z = 3.0

        # Publish to mavros
        pub.publish(local_pose)

        rate.sleep()
