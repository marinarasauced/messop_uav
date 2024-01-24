import rospy
import mavros_msgs
# import geometry_msgs
from mavros_msgs.srv import CommandBool


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


def arm(input):
    rospy.wait_for_service('/mavros/cmd/arming', timeout=30)
    try:
        cmd_arm = mavros_msgs.srv.CommandBoolRequest()
        cmd_arm.value = input
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
    for _ in range(10):  # Publish for 1 second
        position_target_pub.publish(desired_position)
        rate.sleep()


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

    # Wait to allow takeoff command to complete
    rospy.sleep(15)

    # move_to_position(10, 4, 3)

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
