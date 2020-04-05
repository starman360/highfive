#!/usr/bin/env python
import rospy, time, math
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
import tf

record = 0
current_time = time.localtime()
filename = str(current_time.tm_year) + str(current_time.tm_mon) + str(current_time.tm_mday) + "_" + str(current_time.tm_hour) + str(current_time.tm_min) + str(current_time.tm_sec) + ".txt"


def joint_cb(js):
    global record, filename
    if(record):
        t = rospy.get_rostime()
        s = "" + str(t) + ", " + str(js.position[2]) + ", " + str(js.position[1]) + ", " + str(js.position[0]) + ", " + str(js.position[3]) + ", " + str(js.position[4]) + ", " + str(js.position[5]) + "\n"
        with open(filename, 'a') as f:
            f.write(s)

def record_cb(r):
    global record
    record = r.data
    if record:
        print("RECORDING")
        current_time = time.localtime()
        filename = str(current_time.tm_year) + str(current_time.tm_mon) + str(current_time.tm_mday) + "_" + str(current_time.tm_hour) + str(current_time.tm_min) + str(current_time.tm_sec) + ".txt"
        with open(filename, 'a') as f:
            f.write("t, j0, j1, j2, j3, j4, j5\n")
    else:
        print("STOPPED")

def record_js():

    # with open(filename, 'a') as f:
    #     f.write("t, j0, j1, j2, j3, j4, j5\n")

    rospy.init_node('recordjs', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, joint_cb)
    rospy.Subscriber("/is_recording", Int64, record_cb)

    listener = tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            (trans0,rot0) = listener.lookupTransform('/shoulder_link', '/base_link', rospy.Time(0))
            (trans1,rot1) = listener.lookupTransform('/upper_arm_link', '/shoulder_link', rospy.Time(0))
            (trans2,rot2) = listener.lookupTransform('/forearm_link', '/upper_arm_link', rospy.Time(0))
            (trans3,rot3) = listener.lookupTransform('/wrist_1_link', '/forearm_link', rospy.Time(0))
            (trans4,rot4) = listener.lookupTransform('/wrist_2_link', '/wrist_1_link', rospy.Time(0))
            (trans5,rot5) = listener.lookupTransform('/wrist_3_link', '/wrist_2_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rospy.spin()

if __name__ == '__main__':
    try:
        record_js()
    except rospy.ROSInterruptException:
        pass