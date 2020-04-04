#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState

record = 0
filename = "data.txt"

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
        with open(filename, 'a') as f:
            f.write("t, j0, j1, j2, j3, j4, j5\n")
    else:
        print("STOPPED")

def record_js():

    with open(filename, 'a') as f:
        f.write("t, j0, j1, j2, j3, j4, j5\n")

    rospy.init_node('recordjs', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, joint_cb)
    rospy.Subscriber("/is_recording", Int64, record_cb)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        record_js()
    except rospy.ROSInterruptException:
        pass