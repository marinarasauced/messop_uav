from vicon_integration import StateUAV
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
import rospy
import time

import threading


if __name__ == "__main__":
    # rospy.init_node("UAV_node", anonymous=True)

    local_state = StateUAV()

    rospy.wait_for_message("/vicon/hawk2/hawk2", TransformStamped)
    print("VICON data received")

    pose_thread = threading.Thread(target=local_state.vicon_subscriber)
    pose_thread.start()

    rospy.wait_for_message("/mavros/local_position/pose", PoseStamped)
    print("Position data received")

    rospy.Subscriber(
        "/mavros/local_position/pose", PoseStamped, local_state.save_local_position
    )
    while not rospy.is_shutdown():
        print(local_state.Px, local_state.Py, local_state.Pz)
        time.sleep(5)