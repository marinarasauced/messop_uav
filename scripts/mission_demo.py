import rospy
import mavros_msgs
import geometry_msgs
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from vicon_integration import StateUAV
import time

# TODO: add a "geofence" or interrupt to bound the fly zone of the UAV
# TODO: add support for if VICON feedback is dropped


def set_mode():
    rospy.wait_for_service('/mavros/set_mode', timeout=30)
    try:
        change_mode = mavros_msgs.srv.SetModeRequest()
        change_mode.custom_mode = 'GUIDED'
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

        while local_state.Pz - altitude < local_state.error_tol:
            print('UAV is taking off...')

        return response.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def move_to_position(x, y, z):
    # Create a mavros PositionTarget message
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
    while (local_state.Px ** 2 + local_state.Py ** 2 + local_state.Pz ** 2) ** (0.5) < local_state.error_tol:
        print("UAV is moving to waypoint")

    return True


def land():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        cmd_land = mavros_msgs.srv.CommandTOLRequest
        land_request = cmd_land(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
        service_caller = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        response = service_caller(land_request)
        return response.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    rospy.init_node('UAV_node', anonymous=True)

    local_state = StateUAV
    # Update the position data:
    # local_state.x, local_state.y, and local_state.z are now available for all functions
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, local_state.publish_local_position)

    # Set mode to GUIDED
    if set_mode():
        print('Vehicle is in GUIDED mode')
        # Arm the vehicle
        if arm(True):
            print("Vehicle armed")

            # Specify the takeoff altitude (in meters)
            takeoff_altitude = 1.0

            # Takeoff
            if takeoff(takeoff_altitude):
                print("Vehicle taking off to {} meters".format(takeoff_altitude))
            else:
                print("Takeoff failed")
        else:
            print("Arming failed")
    else:
        print('Set mode failed')

    if move_to_position(10, 4, 3):
        print("UAV has reached waypoint")
    else:
        print("Move command has failed")

    # Land
    if land():
        print("Vehicle landed")
    else:
        print("Land failed")

    # Wait to allow land command to finish
    rospy.sleep(15)

    # Disarm the motors is not already done
    if arm(False):
        print('Vehicle disarmed')
    else:
        print('Disarm failed: vehicle still armed')
