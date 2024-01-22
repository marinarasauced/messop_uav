import rospy
from mavros_msgs.srv import CommandBool, CommandTOL
from mavros_msgs.srv import CommandBoolRequest, CommandBoolResponse, CommandTOLRequest, CommandTOLResponse
import mavros_msgs


def set_mode():
    rospy.wait_for_service('/mavros/set_mode', timeout=30)
    try:
        data = mavros_msgs.srv.SetModeRequest()
        data.custom_mode = 'GUIDED'
        service_caller = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        service_caller(data)
        return 1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def arm():
    rospy.wait_for_service('/mavros/cmd/arming', timeout=30)
    try:
        data = mavros_msgs.srv.CommandBoolRequest()
        data.value = True
        service_caller = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        service_caller(data)
        return 1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def takeoff(altitude):
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        takeoff_request = CommandTOLRequest(altitude=altitude, latitude=0, longitude=0, min_pitch=0, yaw=0)
        response = takeoff_service(takeoff_request)
        return 1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    rospy.init_node('takeoff_node', anonymous=True)

    # Set mode to GUIDED
    if set_mode() == 1:
        print('Vehicle is in GUIDED mode')
        # Arm the vehicle
        if arm() == 1:
            print("Vehicle armed")

            # Specify the takeoff altitude (in meters)
            takeoff_altitude = 5.0

            # Takeoff
            if takeoff(takeoff_altitude) == 1:
                print("Vehicle taking off to {} meters".format(takeoff_altitude))
            else:
                print("Takeoff failed")
        else:
            print("Arming failed")
    else:
        print('Set mode failed')
