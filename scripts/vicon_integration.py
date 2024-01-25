import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import ParamSet


class StateUAV:
    def __init__(self):
        # The codes knowledge of where the UAV is
        self.Px = None
        self.Py = None
        self.Pz = None

        # Basic components: publisher and error tolerance
        # Publisher is used to update the position of the UAV
        # Error tolerance is used to monitor when the vehicle has reached its destination
        # Time_vicon is the measurement of when the vicon data was last received
        # Time_odometry
        self.pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=10)
        self.error_tol = 0.1    # in meters
        self.time_vicon = None

        # limits of where the UAV can fly:
        self.x_lim = None
        self.y_lim = None
        self.z_lim = None

    @staticmethod
    def set_odometry_parameters():
        rospy.wait_for_service('/mavros/param/set')
        param_set_service = rospy.ServiceProxy('/mavros/param/set', ParamSet)

        # set the priority for /mavros/odometry/out
        param_set_service(param_id='EK3_SRC1_POSZ', value=[1, 0.0])

        # set the priority for /mavros/odometry/in
        param_set_service(param_id='EK3_SRC2_POSZ', value=[1, 1.0])

    def publish_local_position(self, position):
        # Constantly update the location of the UAV

        # Publish the position to mavros
        self.pub.publish(position)

        # update the local position variable:
        self.Px = position.pose.position.x      # local_pose.pose.position.x
        self.Py = position.pose.position.y      # local_pose.pose.position.y
        self.Pz = position.pose.position.z      # local_pose.pose.position.z


