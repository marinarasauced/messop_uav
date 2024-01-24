import rospy
from geometry_msgs.msg import PoseStamped


class StateUAV:
    def __init__(self):
        self.Px = None
        self.Py = None
        self.Pz = None
        self.error_tol = 0.2    # in meters

    def publish_local_position(self, position):
        # rospy.init_node('local_position_publisher', anonymous=True)
        pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=10)

        # rate = rospy.Rate(10)  # Use this to dictate the frequency that location is published

        # Constantly update the location of the UAV
        # while not rospy.is_shutdown():
        # Create a PoseStamped message
        local_pose = PoseStamped()
        local_pose.header.stamp = rospy.Time.now()
        local_pose.header.frame_id = 'map'  # Indicate that coordinates are global

        # Set local position
        local_pose.pose.position.x = position.pose.position.x
        local_pose.pose.position.y = position.pose.position.y
        local_pose.pose.position.z = position.pose.position.z

        # Publish to mavros
        pub.publish(position)

        # update the local position variable:
        self.Px = position.pose.position.x      # local_pose.pose.position.x
        self.Py = position.pose.position.y      # local_pose.pose.position.y
        self.Pz = position.pose.position.z      # local_pose.pose.position.z

            # rate.sleep()
