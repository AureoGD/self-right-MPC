#!/usr/bin/env python
import rospy
from hyq2max_joints_position_controller.msg import HyQ2max_joints
from hyq2max_joints_position_controller.msg import HyQ2max_command
from std_msgs.msg import Float32MultiArray
import std_msgs.msg
import numpy as np

class msgfrommatlab():

    def __init__(self):
        rospy.init_node('msgFROMmatlab')

        rospy.Subscriber("/matlab2ros",
                         Float32MultiArray,
                         self.matlab2ros)

        self.pub_joints_ref = rospy.Publisher('/hyq2max/HyQ2maxJointsPositionController/command',
                                              HyQ2max_command,
                                              queue_size=10)

        self.q = None
        self.qd = None
        self.refs = HyQ2max_command()

    def joint_command(self, q_r, qd_r):
        self.refs.pos_command = q_r
        self.refs.vel_command = qd_r
        self.pub_joints_ref.publish(self.refs)

    def matlab2ros(self, data):
        q = data.data[0:12]
        qd = data.data[12:24]
        print q, qd
        self.joint_command(q, qd)

    def run(self):
        while not rospy.is_shutdown():
            pass


if __name__ == "__main__":
    node = msgfrommatlab()
    node.run()