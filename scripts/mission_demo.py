import rospy
import mavros_msgs
import geometry_msgs
from geometry_msgs.msg import PoseStamped, QuaternionStamped
from mavros_msgs.srv import CommandBool
from vicon_integration import StateUAV
import time
from mavros_msgs.srv import ParamSet


def set_mode(mode):
    rospy.wait_for_service('/mavros/set_mode', timeout=30)
    try:
        change_mode = mavros_msgs.srv.SetModeRequest()
        change_mode.custom_mode = mode
        service_caller = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        service_caller(change_mode)
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def arm(status):
    rospy.wait_for_service('/mavros/cmd/arming', timeout=30)
    try:
        cmd_arm = mavros_msgs.srv.CommandBoolRequest()
        cmd_arm.value = status
        service_caller = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        service_caller(cmd_arm)
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def takeoff(altitude):
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        cmd_takeoff = mavros_msgs.srv.CommandTOLRequest
        takeoff_request = cmd_takeoff(altitude=altitude, latitude=0, longitude=0, min_pitch=0, yaw=0)
        service_caller = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
        response = service_caller(takeoff_request)

        while abs(local_state.Pz - altitude) > local_state.error_tol:
            pass

        return response.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def move_to_position(x, y, z):
    # Create a mavros PositionTarget message
    if (x < local_state.x_lim) & (y < local_state.y_lim) & (z < local_state.z_lim):
        desired_position = mavros_msgs.msg.PositionTarget()
        desired_position.coordinate_frame = desired_position.FRAME_LOCAL_NED
        desired_position.type_mask = desired_position.IGNORE_VX | desired_position.IGNORE_VY | desired_position.IGNORE_VZ | \
                                     desired_position.IGNORE_AFX | desired_position.IGNORE_AFY | desired_position.IGNORE_AFZ | \
                                     desired_position.FORCE | desired_position.IGNORE_YAW | desired_position.IGNORE_YAW_RATE

        # Set the target position
        desired_position.position.x = x
        desired_position.position.y = y
        desired_position.position.z = z

        # Set the target orientation (optional)
        desired_position.yaw = 0.0  # Set the desired yaw angle in radians

        # Publish the PositionTarget message
        position_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', mavros_msgs.msg.PositionTarget, queue_size=10)
        rate = rospy.Rate(10)  # 10 Hz
        while ((local_state.Px - x) ** 2 + (local_state.Py - y) ** 2 + (local_state.Pz - z) ** 2) ** (0.5) > local_state.error_tol:
            position_target_pub.publish(desired_position)
            pass

        return True

    else:
        print(x, y, z, local_state.x_lim, local_state.y_lim, local_state.z_lim)
        print('Destination point is invalid')


def land():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        cmd_land = mavros_msgs.srv.CommandTOLRequest
        land_request = cmd_land(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
        service_caller = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        response = service_caller(land_request)

        while abs(local_state.Pz - 0) > local_state.error_tol:
            pass

        return True

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    rospy.init_node('UAV_node', anonymous=True)

    local_state = StateUAV()

    # local_state.set_odometry_parameters()

    local_state.x_lim = 100
    local_state.y_lim = 100
    local_state.z_lim = 100
    # Update the position data:
    # local_state.Px, local_state.Py, and local_state.Pz are now available for all functions
    rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
    print('Position data received')
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, local_state.publish_local_position)

    while local_state.Px is None:
        print('Waiting for position data')
        time.sleep(1)

    #
    time.sleep(5)

    # rospy.wait_for_message('/mavros/imu/data', PoseStamped)
    # print('Position data received')
    # rospy.Subscriber('/mavros/local_position/pose', PoseStamped, local_state.publish_local_position)
    #
    # while local_state.Px is None:
    #     print('Waiting for position data')
    #     time.sleep(1)

    # Set mode to GUIDED
    if set_mode('GUIDED'):
        print('Vehicle is in GUIDED mode')
        # Arm the vehicle
        if arm(True):
            print("Vehicle armed")
        else:
            print("Arming failed")
    else:
        print('Set mode failed')

    # Specify the takeoff altitude (in meters)
    takeoff_altitude = 10.0
    takeoff(takeoff_altitude)
    print('Vehicle is at: ', takeoff_altitude, ' m')

    move_to_position(24, 30, 20)
    print("UAV has reached waypoint")

    # Land
    if land():
        print("Vehicle landed")
    else:
        print("Land failed")

    # Disarm the motors
    # if arm(False):
    #     print('Vehicle disarmed')
    # else:
    #     print('Disarm failed: vehicle still armed')
