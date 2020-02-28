#!/usr/bin/env python
import rospy
from hyq2max_joints_position_controller.msg import HyQ2max_joints
from hyq2max_joints_position_controller.msg import HyQ2max_command
from std_msgs.msg import Float32MultiArray
import std_msgs.msg
import numpy as np

class msg2matlab():

    def __init__(self):
        rospy.init_node('msg2matlab')

        self.data2matlab = Float32MultiArray()

        rospy.Subscriber("/hyq2max/HyQ2maxJointsPositionController/state_joint",
                         HyQ2max_joints,
                         self.joints_states_callback)

        self.q = None
        self.qd = None

        self.order = 15.0
        self.q2filter = None
        self.qd2filter = None

        self.qarray = np.zeros((12, int(self.order)), dtype=np.float64)
        self.qdarray = np.zeros((12,  int(self.order)), dtype=np.float64)

        # self.pub_joints_ref = rospy.Publisher('/hyq2max/HyQ2maxJointsPositionController/command',
        #                                       HyQ2max_command,
        #                                       queue_size=10)
        #
        self.pub_joints2matlab = rospy.Publisher('/ros2matlab', Float32MultiArray, queue_size=10)

        # self.refs = HyQ2max_command()

    # def joint_command(self, q_r, qd_r):
    #     self.refs.pos_command = q_r
    #     self.refs.vel_command = qd_r
    #     self.pub_joints_ref.publish(self.refs)

    def joints_states_callback(self, data):
        self.q2filter = np.asarray(data.joints_pos).reshape(12, 1)
        self.qarray = np.roll(self.qarray, 1)
        self.qarray[:, 0] = self.q2filter[:, 0]
        self.q = 1/self.order * (np.sum(self.qarray, axis=1)).reshape(12, 1)

        self.qd2filter = np.asarray(data.joints_vel).reshape(12, 1)
        self.qdarray = np.roll(self.qdarray, 1)
        self.qdarray[:, 0] = self.qd2filter[:, 0]
        self.qd = 1/self.order * (np.sum(self.qdarray, axis=1)).reshape(12, 1)


    def send2matlab(self):
        if self.q is not None and self.qd is not None:

            self.data2matlab.data = [self.q[0], self.q[1], self.q[2],
                                     self.q[3], self.q[4], self.q[5],
                                     self.q[6], self.q[7], self.q[8],
                                     self.q[9], self.q[10], self.q[11],
                                     self.qd[0], self.qd[1], self.qd[2],
                                     self.qd[3], self.qd[4], self.qd[5],
                                     self.qd[6], self.qd[7], self.qd[8],
                                     self.qd[9], self.qd[10], self.qd[11]]
            self.pub_joints2matlab.publish(self.data2matlab)


    def run(self):
        while not rospy.is_shutdown():
            self.send2matlab()


if __name__ == "__main__":
    node = msg2matlab()
    node.run()