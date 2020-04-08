#!/usr/bin/env python
import rospy, time, math
import numpy as np
from numpy import linalg as LA
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
import tf2_ros
from ft import ft

record = 0
current_time = time.localtime()
filename = str(current_time.tm_year) + str(current_time.tm_mon) + str(current_time.tm_mday) + "_" + str(current_time.tm_hour) + str(current_time.tm_min) + str(current_time.tm_sec) + ".txt"
tnow = 0

def joint_cb(js):
    global record, filename, jointloc, tnow
    if(record):
        t = rospy.get_rostime().to_sec() - tnow.to_sec()

        s = "" + str(t) + ", " + str(js.position[2]) + ", " + str(js.position[1]) + ", " + str(js.position[0]) + ", " + str(js.position[3]) + ", " + str(js.position[4]) + ", " + str(js.position[5]) + ", " + np.array2string(jointloc,separator=',', suppress_small=True, max_line_width=np.inf)[1:-1]

        with open(filename, 'a') as f:
            f.write(s)
            f.write('\n')

def record_cb(r):
    global record, filename, tnow
    record = r.data
    if record:
        print("RECORDING")
        tnow = rospy.get_rostime()
        current_time = time.localtime()
        filename = str(current_time.tm_year) + str(current_time.tm_mon) + str(current_time.tm_mday) + "_" + str(current_time.tm_hour) + str(current_time.tm_min) + str(current_time.tm_sec) + ".txt"
        with open(filename, 'a') as f:
            f.write("t, j0, j1, j2, j3, j4, j5, l0x, l0y, l0z, l1x, l1y, l1z, l2x, l2y, l2z, l3x, l3y, l3z, l4x, l4y, l4z, l5x, l5y, l5z, l6x, l6y, l6z, ed\n")
    else:
        print("STOPPED")

def record_js():

    # with open(filename, 'a') as f:
    #     f.write("t, j0, j1, j2, j3, j4, j5\n")
    global jointloc, tnow
    rospy.init_node('recordjs', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, joint_cb)
    rospy.Subscriber("/is_recording", Int64, record_cb)


    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():
        try:
            trans0 = tfBuffer.lookup_transform('shoulder_link', 'base_link', rospy.Time())
            trans1 = tfBuffer.lookup_transform('upper_arm_link', 'shoulder_link', rospy.Time())
            trans2 = tfBuffer.lookup_transform('forearm_link', 'upper_arm_link', rospy.Time())
            trans3 = tfBuffer.lookup_transform('wrist_1_link', 'forearm_link', rospy.Time())
            trans4 = tfBuffer.lookup_transform('wrist_2_link', 'wrist_1_link', rospy.Time())
            trans5 = tfBuffer.lookup_transform('wrist_3_link', 'wrist_2_link', rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

        # print("---------------------------")
        A0_0 = ft.from_tf(trans0)
        A0_1 = np.dot(A0_0,ft.from_tf(trans1))
        A0_2 = np.dot(A0_1,ft.from_tf(trans2))
        A0_3 = np.dot(A0_2,ft.from_tf(trans3))
        A0_4 = np.dot(A0_3,ft.from_tf(trans4))
        A0_5 = np.dot(A0_4,ft.from_tf(trans5))
        goal = np.array([0.65184934, 0.11281376, 0.71111751])
        ed = LA.norm(A0_5[0:3,3].reshape(1,3)-goal)

        jointloc = np.concatenate((A0_0[0:3,3], A0_1[0:3,3], A0_2[0:3,3], A0_3[0:3,3], A0_4[0:3,3], A0_5[0:3,3], ed), axis=None)
        # print('-------------------')
        # print(rospy.get_rostime().to_sec())
        # print(tnow.to_sec())
        # print(rospy.get_rostime().to_sec() - tnow.to_sec())
        # print(np.array2string(jointloc,separator=',', suppress_small=True, max_line_width=np.inf)[1:-1] + '\n\n')

if __name__ == '__main__':
    try:
        record_js()
    except rospy.ROSInterruptException:
        pass