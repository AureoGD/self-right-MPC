#!/usr/bin/env python
"""
Aureo Guilherme Dobrikopf,
Master student at Santa Catarina State University(UDESC)
aureogd@gmail.com

Version 0.0.1
31/10/2019
"""
import rospy
from updateKin import updateKin
from hyq2max_joints_position_controller.msg import HyQ2max_joints
from hyq2max_joints_position_controller.msg import HyQ2max_command
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ContactsState
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_matrix
from std_srvs.srv import Empty, EmptyRequest
import os
import rospkg
import numpy as np
import osqp
from DynModel import DynModel
from math import sin, cos

import time

from ParameterMPC import ParametersMPC


class SelfRightPkg():

    def __init__(self):

        self.dinamic = DynModel()
        self.inte = 1
        rospy.init_node('SelfRight')

        rospy.Subscriber("/hyq2max/HyQ2maxJointsPositionController/state_joint",
                         HyQ2max_joints,
                         self.joints_states_callback)

        rospy.Subscriber("/hyq2max/ground_truth",
                         Odometry,
                         self.pose_callback)

        rospy.Subscriber("/lf_grf_sensor_state",
                         ContactsState,
                         self.lf_ft_cs_callback)

        rospy.Subscriber("/rf_grf_sensor_state",
                         ContactsState,
                         self.rf_ft_cs_callback)

        rospy.Subscriber("/lh_grf_sensor_state",
                         ContactsState,
                         self.lh_ft_cs_callback)

        rospy.Subscriber("/rh_grf_sensor_state",
                         ContactsState,
                         self.rh_ft_cs_callback)

        """Positon and speed variables"""
        self.position = None
        self.orientation = None
        self.twist = None

        """Joints states"""
        self.q = None
        self.qd = None

        """Fallen check variable"""
        self.Z_fall = 0.2
        self.Z_stand = 0.7
        self.FLAG_fall = False
        self.pitch = 0

        """Foot contacts"""
        self.lf_ft_contact = None
        self.rf_ft_contact = None
        self.lh_ft_contact = None
        self.rh_ft_contact = None

        self.pub_joints_ref = rospy.Publisher('/hyq2max/HyQ2maxJointsPositionController/command',
                                              HyQ2max_command,
                                              queue_size=10)

        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        rospack = rospkg.RosPack()

        # with open(os.path.join(rospack.get_path("self_right_ctrl"), "src", "q_ref.npy")) as f:
        #     self.q_ref = np.load('q_ref.npy')
        # with open(os.path.join(rospack.get_path("self_right_ctrl"), "src", "qd_ref.npy")) as f:
        #     self.qd_ref = f.read()

        self.q_ref = np.load(os.path.join(rospack.get_path("self_right_ctrl"), "src", "q_ref.npy"))
        self.qd_ref = np.load(os.path.join(rospack.get_path("self_right_ctrl"), "src", "qd_ref.npy"))

        self.q_ref_st = np.load(os.path.join(rospack.get_path("self_right_ctrl"), "src", "q_st_ref.npy"))
        self.qd_ref_st = np.load(os.path.join(rospack.get_path("self_right_ctrl"), "src", "qd_st_ref.npy"))

        self.refs = HyQ2max_command()

        self.st0_flag = False
        self.st1_flag = False
        self.st2_flag = False
        self.st3_flag = False
        self.st4_flag = False
        self.st5_flag = False
        self.st6_flag = False

        self.q_sp = np.array([0.0, 1.65, -2.85, 0.0, 1.65, -2.85, 0.0, -1.65, 2.85, 0.0, -1.65, 2.85]).reshape((12, 1))
        self.q_pp = np.array([-0.4, 2.3, -2.85, -0.4,  2.3, -2.85, -0.4, -2.3, 2.85, -0.4, -2.3, 2.85]).reshape((12, 1))

        self.kin = updateKin()

        self.ref_count = 0

        self.fs = 250.0  # Sample frequency
        self.ts = 1 / (self.fs)

        self.sr_MPC = ParametersMPC(4, 2, self.ts)

        self.fall_check_rate = rospy.Rate(1)
        self.sr_controller = rospy.Rate(self.fs)

        self.data_to_save = ''
        self.f = open("recorddata.txt", "w")
        self.data2s = ''
        self.data_qd = ''

        self.A = np.zeros((12,12),dtype=np.float64)
        self.B = np.zeros((12, 24), dtype=np.float64)
        self.x = np.zeros((12, 1), dtype=np.float64)
        self.u = np.zeros((24, 1), dtype=np.float64)

        """Start and setup of OSQP"""
        self.prob = osqp.OSQP()
        self.sr_MPC.initmodel()
        self.prob.setup(self.sr_MPC.H,
                        np.transpose(self.sr_MPC.F),
                        self.sr_MPC.A_Constraints,
                        self.sr_MPC.b_l,
                        self.sr_MPC.b_u,
                        verbose = True,
                        warm_start=False,
                        alpha=1,
                        adaptive_rho=False)

    def lf_ft_cs_callback(self, data):

        self.lf_ft_contact = data

    def rf_ft_cs_callback(self, data):
        self.rf_ft_contact = data

    def lh_ft_cs_callback(self, data):
        self.lh_ft_contact = data

    def rh_ft_cs_callback(self, data):
        self.rh_ft_contact = data

    def pose_callback(self, data):
        self.position = data.pose.pose.position
        self.orientation = data.pose.pose.orientation
        self.twist = data.twist.twist

    def joints_states_callback(self, data):
        self.q = data.joints_pos
        self.qd = data.joints_vel

    def get_states(self):
        q = np.array([self.q[0], self.q[1], self.q[2],
                      self.q[3], self.q[4], self.q[5],
                      self.q[6], self.q[7], self.q[8],
                      self.q[9], self.q[10], self.q[11]]).reshape((12, 1))

        qd = np.array([self.twist.angular.x, self.twist.angular.y, self.twist.angular.z,
                       self.twist.linear.x, self.twist.linear.y, self.twist.linear.z,
                       self.qd[0], self.qd[1], self.qd[2],
                       self.qd[3], self.qd[4], self.qd[5],
                       self.qd[6], self.qd[7], self.qd[8],
                       self.qd[9], self.qd[10], self.qd[11]]).reshape(18, 1)

        orientation_list = [self.orientation.x, self.orientation.y,
                            self.orientation.z, self.orientation.w]

        rot_mtx =  quaternion_matrix(orientation_list)[0:3,0:3]

        com_pos = np.array([self.position.x, self.position.y, self.position.z]).reshape(3, 1)


        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        roll = float("{0:.2f}".format(float(roll)))
        pitch = float("{0:.2f}".format(float(pitch)))
        yaw = float("{0:.2f}".format(float(yaw)))

        #print roll, pitch, yaw

        con_list = [False, False, False, False]
        con_p = np.full((12, 1), np.nan,dtype=np.float64)
        sum_f = np.zeros((3, 1), dtype=np.float64)
        sum_t = np.zeros((3, 1), dtype=np.float64)

        # con_p[0:3] = com_pos + np.matmul(rot_mtx, self.kin.updateHT_LF_foot(q).reshape(3,1))
        # con_p[3:6] = com_pos + np.matmul(rot_mtx,self.kin.updateHT_RF_foot(q).reshape(3,1))
        # con_p[6:9] = com_pos + np.matmul(rot_mtx,self.kin.updateHT_LH_foot(q).reshape(3,1))
        # con_p[9:12] = com_pos + np.matmul(rot_mtx,self.kin.updateHT_RH_foot(q).reshape(3,1))
        #
        # if con_p[2] <= 0.06:
        #     con_list[0] = True
        # if con_p[5] <= 0.06:
        #     con_list[1] = True
        # if con_p[8] <= 0.06:
        #     con_list[2] = True
        # if con_p[11] <= 0.06:
        #     con_list[3] = True

        if not self.lf_ft_contact.states:
            pass
        else:
            con_list[0] = True
            try:
                con_p[0, 0] = float("{0:.2f}".format(float(self.lf_ft_contact.states[0].contact_positions[0].x-self.position.x)))
                con_p[1, 0] = float("{0:.2f}".format(float(self.lf_ft_contact.states[0].contact_positions[0].y-self.position.y)))
                con_p[2, 0] = float("{0:.2f}".format(float(self.lf_ft_contact.states[0].contact_positions[0].z-self.position.z)))

                sum_t[0, 0] = sum_t[0, 0] + self.lf_ft_contact.states[0].total_wrench.torque.x
                sum_t[1, 0] = sum_t[1, 0] + self.lf_ft_contact.states[0].total_wrench.torque.y
                sum_t[2, 0] = sum_t[2, 0] + self.lf_ft_contact.states[0].total_wrench.torque.z

                sum_f[0, 0] = sum_f[0, 0] + self.lf_ft_contact.states[0].total_wrench.force.x
                sum_f[1, 0] = sum_f[1, 0] + self.lf_ft_contact.states[0].total_wrench.force.y
                sum_f[2, 0] = sum_f[2, 0] + self.lf_ft_contact.states[0].total_wrench.force.z

            except:
                print "lf", self.lf_ft_contact

        if not self.rf_ft_contact.states:
            pass
        else:
            con_list[1] = True
            try:
                con_p[3, 0] = float("{0:.2f}".format(float(self.rf_ft_contact.states[0].contact_positions[0].x-self.position.x)))
                con_p[4, 0] = float("{0:.2f}".format(float(self.rf_ft_contact.states[0].contact_positions[0].y-self.position.y)))
                con_p[5, 0] = float("{0:.2f}".format(float(self.rf_ft_contact.states[0].contact_positions[0].z-self.position.z)))

                sum_t[0, 0] = sum_t[0 ,0] + self.rf_ft_contact.states[0].total_wrench.torque.x
                sum_t[1, 0] = sum_t[1, 0] + self.rf_ft_contact.states[0].total_wrench.torque.y
                sum_t[2, 0] = sum_t[2, 0] + self.rf_ft_contact.states[0].total_wrench.torque.z

                sum_f[0, 0] = sum_f[0, 0] + self.rf_ft_contact.states[0].total_wrench.force.x
                sum_f[1, 0] = sum_f[1, 0] + self.rf_ft_contact.states[0].total_wrench.force.y
                sum_f[2, 0] = sum_f[2, 0] + self.rf_ft_contact.states[0].total_wrench.force.z
            except:
                print "rf", self.rf_ft_contact


        if not self.lh_ft_contact.states:
            pass
        else:
            con_list[2] = True
            try:
                con_p[6, 0] = float("{0:.2f}".format(float(self.lh_ft_contact.states[0].contact_positions[0].x-self.position.x)))
                con_p[7, 0] = float("{0:.2f}".format(float(self.lh_ft_contact.states[0].contact_positions[0].y-self.position.y)))
                con_p[8, 0] = float("{0:.2f}".format(float(self.lh_ft_contact.states[0].contact_positions[0].z-self.position.z)))

                sum_t[0, 0] = sum_t[0, 0] + self.lh_ft_contact.states[0].total_wrench.torque.x
                sum_t[1, 0] = sum_t[1, 0] + self.lh_ft_contact.states[0].total_wrench.torque.y
                sum_t[2, 0] = sum_t[2, 0] + self.lh_ft_contact.states[0].total_wrench.torque.z

                sum_f[0, 0] = sum_f[0, 0] + self.lh_ft_contact.states[0].total_wrench.force.x
                sum_f[1, 0] = sum_f[1, 0] + self.lh_ft_contact.states[0].total_wrench.force.y
                sum_f[2, 0] = sum_f[2, 0] + self.lh_ft_contact.states[0].total_wrench.force.z
            except:
                print "lh", self.lh_ft_contact

        if not self.rh_ft_contact.states:
            pass
        else:
            con_list[3] = True
            try:
                con_p[9, 0] = float("{0:.2f}".format(float(self.rh_ft_contact.states[0].contact_positions[0].x-self.position.x)))
                con_p[10, 0] = float("{0:.2f}".format(float(self.rh_ft_contact.states[0].contact_positions[0].y-self.position.y)))
                con_p[11, 0] = float("{0:.2f}".format(float(self.rh_ft_contact.states[0].contact_positions[0].z-self.position.z)))

                sum_t[0, 0] = sum_t[0, 0] + self.rh_ft_contact.states[0].total_wrench.torque.x
                sum_t[1, 0] = sum_t[1, 0] + self.rh_ft_contact.states[0].total_wrench.torque.y
                sum_t[2, 0] = sum_t[2, 0] + self.rh_ft_contact.states[0].total_wrench.torque.z

                sum_f[0, 0] = sum_f[0, 0] + self.rh_ft_contact.states[0].total_wrench.force.x
                sum_f[1, 0] = sum_f[1, 0] + self.rh_ft_contact.states[0].total_wrench.force.y
                sum_f[2, 0] = sum_f[2, 0] + self.rh_ft_contact.states[0].total_wrench.force.z
            except:
                print "rh", self.rh_ft_contact

        return roll, q, qd, con_list, con_p, sum_t, sum_f

    def traj_q(self, q0, qf, tf):

        a_11 = (tf ** 3) / 3
        a_12 = (tf ** 2) / 2
        a_21 = (tf ** 2)
        a_22 = tf

        A = np.array(([[a_11, a_12], [a_21, a_22]]), dtype=np.float64)
        b_11 = (qf - q0)
        coef = np.ndarray((12, 2), dtype=np.float64)
        for i in range(12):
            coef[i] = np.linalg.solve(A, np.array(([[b_11[i]], [0]]), dtype=np.float64)).reshape(1, 2)

        q0 = q0.reshape(1, 12)

        n_traj_p = int(tf / self.ts)

        q_traj = np.ndarray((12, n_traj_p), dtype=np.float64)
        qd_traj = np.ndarray((12, n_traj_p), dtype=np.float64)

        k = 1
        while k != n_traj_p:
            qd_traj[:, k] = (coef[:, 0] * (k * self.ts) ** 2 + coef[:, 1] * k * self.ts)
            q_traj[:, k] = q0[0, :] + (coef[:, 0] * (k * self.ts) ** 3) / 3 + (coef[:, 1] * (k * self.ts) ** 2) / 2
            k = k + 1


        return q_traj, qd_traj, n_traj_p

    def joint_command(self, q_r, qd_r):
        self.refs.pos_command = q_r
        self.refs.vel_command = qd_r
        #self.pause(EmptyRequest())
        #roll, q, qd, con_list, con_p, sum_t, sum_f = self.get_states()
        #self.dinamic.updateall(q, qd)
        #self.data_q = np.matmul(self.dinamic.Ag_sup, qd[0:6]) + np.matmul(self.dinamic.Ag_inf, qd[6:18])
        #self.savedata()
        #self.inte = self.inte + 1
        #self.unpause(EmptyRequest())
        self.pub_joints_ref.publish(self.refs)
        self.sr_controller.sleep()

    def run(self):
        while not rospy.is_shutdown():
            if self.position is not None:
                if self.position.z < self.Z_fall:
                    rospy.loginfo("The robot has been fallen")
                    self.FLAG_fall = True
                    rospy.sleep(2.)
                    #self.savedata()

                    while self.FLAG_fall:

                        # self.pause(EmptyRequest())
                        # self.unpause(EmptyRequest())

                        # """Self right Victor """
                        # self.refs.pos_command = self.q_ref[0:12, k]
                        # self.refs.vel_command = self.qd_ref[0:12, k]
                        # self.pub_joints_ref.publish(self.refs)
                        #
                        # print self.lf_ft_contact.states, self.rf_ft_contact.states, \
                        #     self.lh_ft_contact.states, self.rh_ft_contact.states
                        #
                        # if k == 999 + 250 + 187:
                        #     self.FLAG_fall = False
                        #     self.pause(EmptyRequest())

                        """Update states model"""

                        roll, q, qd, con_list, con_p, sum_t, sum_f = self.get_states()



                        """States"""

                        """ Go to safe position"""
                        if self.st0_flag is False and (roll == 3.14 or roll == -3.14) and (q != self.q_sp).any():
                            print "Go to safe position"
                            q_ref, qd_ref, n_traj_p = self.traj_q(q, self.q_sp, 4)
                            k = 0

                            while k != n_traj_p:
                                self.joint_command(q_ref[:, k], qd_ref[:, k])
                                k = k + 1

                            self.st0_flag = True

                        """ Find right foot contacts"""
                        if self.st1_flag is False and self.st0_flag is True:
                            print "Find right foot contacts"
                            self.st1_flag = True
                            k = 0

                            while k != 499:
                                self.joint_command(self.q_ref[:, k], self.qd_ref[:, k])
                                k = k + 1
                            self.st0_flag = False

                        """ Prepare and roll"""
                        if self.st2_flag is False and self.st1_flag is True and \
                                con_list[1] is True and con_list[3] is True:
                            print "Prepare and roll"
                            # self.pause(EmptyRequest())
                            self.st2_flag = True
                            k = 500

                            while k != 1249:
                                # roll, q, qd, foot_con_lf, foot_con_rf, foot_con_lh, foot_con_rh = self.get_states()
                                # self.sr_MPC.updatemodel(q, qd, 'PrepAndRoll')
                                # self.prob.update(Px=self.sr_MPC.H.data,
                                #                  q=np.transpose(self.sr_MPC.F),
                                #                  Ax=self.sr_MPC.A_Constraints.data,
                                #                  l=self.sr_MPC.b_l, u=self.sr_MPC.b_u)
                                #
                                # """Evaluate the control action"""
                                # resp = self.prob.solve()
                                # if resp.info.status != 'solved':
                                #     #  Unlock simulation
                                #     self.unpause(EmptyRequest())
                                #     raise ValueError('OSQP did not solve the problem!')
                                #
                                # self.joint_command(-q[0:12, 0] + resp.x[0:12] * self.ts + resp.x[12:24] * self.ts ** 2,
                                #                    -qd[0:12, 0] + resp.x[0:12] + self.ts * resp.x[12:24])
                                self.joint_command(self.q_ref[:, k], self.qd_ref[:, k])
                                k = k + 1
                            self.st1_flag = False

                        """ Prepare KFE and HAA"""
                        if self.st3_flag is False and self.st2_flag is True and \
                                con_list[0] is True and con_list[1] is True and \
                                con_list[2] is True and con_list[3] is True:
                            print "Prepare KFE and HAA"
                            self.st3_flag = True
                            q_ref, qd_ref, n_traj_p = self.traj_q(q, self.q_pp, 0.75)
                            k = 0
                            while k != n_traj_p:
                                self.joint_command(q_ref[:, k], qd_ref[:, k])
                                k = k + 1

                            self.st2_flag = False

                        """ Prepare KFE """
                        if self.st4_flag is False and self.st3_flag is True and roll == 0:
                            print "Prepare KFE"
                            self.st4_flag = True
                            k = 1436

                            while k != 1687:
                                self.joint_command(self.q_ref[:, k], self.qd_ref[:, k])
                                k = k + 1

                            self.st3_flag = False

                        """Find contacts"""
                        if self.st5_flag is False and self.st4_flag is True and roll == 0:
                            print "Find contacts"
                            self.st5_flag = True
                            k = 1687

                            while con_list[0] is False and con_list[1] is False\
                                    and con_list[2] is False and con_list[3] is False:
                                roll, q, qd, con_list, con_p, sum_t, sum_f = self.get_states()
                                self.joint_command(self.q_ref[:, k], self.qd_ref[:, k])
                                k = k + 1
                            self.joint_command(self.q_ref[:, k], self.qd_ref[:, k])
                            self.st4_flag = False

                            rospy.sleep(1)

                        """Stand Up"""
                        if self.st6_flag is False and self.st5_flag is True and roll == 0:
                            self.st6_flag = True
                            k = 0

                            # for k in range(800):
                            #     roll, q, qd, con_list, con_p, sum_t, sum_f = self.get_states()
                            #     self.pause(EmptyRequest())
                            #     self.model(q, qd, k)
                            #     self.unpause(EmptyRequest())
                            #     self.joint_command(self.q_ref_st[:, k], self.qd_ref_st[:, k])



                            while self.position.z <= 0.65:

                                roll, q, qd, con_list, con_p, sum_t, sum_f = self.get_states()
                            #     #print sum_f

                                self.pause(EmptyRequest())
                                #print con_list

                                self.sr_MPC.updatemodel(q, qd, 'StandUp', con_list, con_p, sum_t)
                                self.savedata()


                                self.prob.update(Px=self.sr_MPC.H.data,
                                                 q=np.transpose(self.sr_MPC.F),
                                                 Ax=self.sr_MPC.A_Constraints.data,
                                                 l=self.sr_MPC.b_l, u=self.sr_MPC.b_u)

                                """Evaluate the control action"""
                                resp = self.prob.solve()
                                if resp.info.status != 'solved':
                                    # #  Unlock simulation
                                    self.unpause(EmptyRequest())
                                    raise ValueError('OSQP did not solve the problem!')

                                #print np.matmul(self.sr_MPC.A_aux, resp.x), "\n"
                                #print self.sr_MPC.b_u

                                # qd_vel = np.array([-(resp.x[0] + resp.x[12] * self.ts *2),
                                #                   (resp.x[1] + resp.x[13] * self.ts *2),
                                #                   -(resp.x[2] + resp.x[14] * self.ts *2),
                                #                   -(resp.x[3] + resp.x[15] * self.ts *2),
                                #                   (resp.x[4] + resp.x[16] * self.ts *2),
                                #                   -(resp.x[5] + resp.x[17] * self.ts *2),
                                #                   -(resp.x[6] + resp.x[18] * self.ts *2),
                                #                   -(resp.x[7] + resp.x[19] * self.ts *2),
                                #                   (resp.x[8] + resp.x[20] * self.ts *2),
                                #                   -(resp.x[9] + resp.x[21] * self.ts *2),
                                #                   -(resp.x[10] + resp.x[22] * self.ts *2),
                                #                   (resp.x[11] + resp.x[23] * self.ts *2)])

                                #self.data_q = q[0:12, 0] + self.ts * qd_vel
                                #self.data_qd = qd_vel
                                # self.savedata()
                                self.unpause(EmptyRequest())

                                print (q[0:12, 0] + resp.x[0:12] * self.ts + resp.x[12:24] * self.ts ** 2), \
                                    resp.x[0:12] + resp.x[12:24] * self.ts

                                self.joint_command((q[0:12, 0] + resp.x[0:12] * self.ts + resp.x[12:24] * self.ts ** 2),
                                                   resp.x[0:12] * 0 + resp.x[12:24] * self.ts * 0)
                            #
                            #     # self.joint_command(self.q_ref[:, k], self.qd_ref[:, k])
                            #     # k = k + 1
                            #     # self.savedata()
                            self.st5_flag = False

                        if self.st6_flag is True:
                            self.FLAG_fall = False
                            self.st6_flag = False

                else:
                    rospy.loginfo("The robot is standing")
                    if (self.lf_ft_contact is not None) and (self.rf_ft_contact is not None) and (
                            self.lh_ft_contact is not None) and (self.rh_ft_contact is not None):

                        roll, q, qd, con_list, con_p, sum_t, sum_f = self.get_states()
                        #print sum_f, sum_t

                self.fall_check_rate.sleep()
                
    def model(self, q, qd, k):
        
        self.dinamic.updateall(q, qd[6:18])

        Ig_inv = np.linalg.inv(self.dinamic.Ig)
        Igd_inv = -np.matmul(np.matmul(Ig_inv, self.dinamic.Igd), Ig_inv)

        M_0 = np.linalg.inv(np.identity(6) - np.matmul(Ig_inv, self.dinamic.Ag_sup))
        M_1 = np.matmul(M_0, Igd_inv)
        M_2 = np.matmul(np.matmul(self.dinamic.Ag_sup, M_0), Ig_inv)
        M_3 = np.matmul(M_0, Ig_inv)

        self.A[0:6, 0:6] = np.identity(6) + self.ts * (np.matmul(self.dinamic.Ag_sup, M_1))
        self.A[0:6, 6:12] = self.ts * (self.dinamic.Agd_sup + np.matmul(M_2, self.dinamic.Agd_sup)) #
        self.A[6:12, 0:6] = Ig_inv + self.ts * M_1
        self.A[6:12, 6:12] = self.ts * np.matmul(M_3, self.dinamic.Agd_sup)

        self.B[0:6, 0:12] = self.ts * (self.dinamic.Agd_inf + np.matmul(M_2, self.dinamic.Agd_inf))
        self.B[0:6, 12:24] = self.ts * (self.dinamic.Ag_inf + np.matmul(M_2, self.dinamic.Ag_inf))
        self.B[6:12, 0:12] = self.ts * np.matmul(M_3, self.dinamic.Agd_inf)
        self.B[6:12, 12:24] = self.ts * np.matmul(M_3, self.dinamic.Ag_inf)

        self.x[0:6] = np.matmul(self.dinamic.Ag_sup, qd[0:6]) + np.matmul(self.dinamic.Ag_inf, qd[6:18])
        self.x[6:12] = qd[0:6]
        self.u[0:12, 0] = self.qd_ref_st[:, k]
        if k == 0:
            self.u[12:24, 0] = ( self.qd_ref_st[:, k] - self.qd_ref_st[:, k] )/ self.ts
        else:
            self.u[12:24, 0] = (self.qd_ref_st[:, k] - self.qd_ref_st[:,k - 1] )/ self.ts

        self.data2s = np.matmul(self.A, self.x) + np.matmul(self.B, self.u)
        self.savedata()





    def savedata(self):

    #     for i in range(len(self.data2s)):
    #         self.f.write(str(float("{0:.5f}".format(float(self.data2s[i]))))+", ")
    # # for i in range(len(self.data_q)):
    # #     self.f.write(str(float("{0:.5f}".format(float(self.data_qd[i]))))+", ")
    #     self.f.write("\n")

        A = self.sr_MPC.A
        B = self.sr_MPC.B
        C = self.sr_MPC.C

        self.f.write(str(self.sr_MPC.count) + '\n')
        self.f.write('\n' + 'A' + '\n')
        for i in range(A.shape[0]):
            for j in range(A.shape[1]):
                self.f.write(str(A[i, j]) + ',')
            self.f.write('\n')

        self.f.write('\n' + 'B' + '\n')
        for i in range(B.shape[0]):
            for j in range(B.shape[1]):
                self.f.write(str(B[i, j]) + ',')
            self.f.write('\n')

        self.f.write('\n' + 'C' + '\n')
        self.f.write(str(self.sr_MPC.count) + '\n')
        for i in range(C.shape[0]):
            for j in range(C.shape[1]):
                self.f.write(str(C[i, j]) + ',')
            self.f.write('\n')
        self.f.write('\n')

        G = self.sr_MPC.G
        H = self.sr_MPC.H_aux
        phi = self.sr_MPC.Phi
        x = self.sr_MPC.x
        F = self.sr_MPC.F
        A_i = self.sr_MPC.A_aux
        b_u = self.sr_MPC.b_u
        b_l = self.sr_MPC.b_l
        ref = self.sr_MPC.reference

        self.f.write('\n' + 'G' + '\n')
        for i in range(G.shape[0]):
            for j in range(G.shape[1]):
                self.f.write(str(G[i, j]) + ',')
            self.f.write('\n')

        self.f.write('\n' + 'H' + '\n')
        for i in range(H.shape[0]):
            for j in range(H.shape[1]):
                self.f.write(str(H[i, j]) + ',')
            self.f.write('\n')

        self.f.write('\n' + 'phi' + '\n')
        for i in range(phi.shape[0]):
            for j in range(phi.shape[1]):
                self.f.write(str(phi[i, j]) + ',')
            self.f.write('\n')

        self.f.write('\n' + 'x' + '\n')
        for i in range(x.shape[0]):
            for j in range(x.shape[1]):
                self.f.write(str(x[i, j]) + ',')
            self.f.write('\n')

        self.f.write('\n' + 'ref' + '\n')
        for i in range(ref.shape[0]):
            for j in range(ref.shape[1]):
                self.f.write(str(ref[i, j]) + ',')
            self.f.write('\n')

        self.f.write('\n' + 'F' + '\n')
        for i in range(F.shape[0]):
            for j in range(F.shape[1]):
                self.f.write(str(F[i, j]) + ',')
            self.f.write('\n')

        self.f.write('\n' + 'A_i' + '\n')
        self.f.write(str(self.sr_MPC.count)+'\n')
        for i in range(A_i.shape[0]):
            for j in range(A_i.shape[1]):
                self.f.write(str(A_i[i, j]) + ',')
            self.f.write('\n')

        self.f.write('\n' + 'b_u' + '\n')
        for i in range(b_u.shape[0]):
            for j in range(b_u.shape[1]):
                self.f.write(str(b_u[i, j]) + ',')
            self.f.write('\n')

        self.f.write('\n'+'b_l'+'\n')
        for i in range(b_l.shape[0]):
            for j in range(b_l.shape[1]):
                self.f.write(str(b_l[i, j]) + ',')
            self.f.write('\n')

        # self.f.write(
        #     str(float("{0:.2f}".format(float(self.twist.angular.x)))) + "," +
        #     str(float("{0:.2f}".format(float(self.twist.angular.y)))) + "," +
        #     str(float("{0:.2f}".format(float(self.twist.angular.z)))) + "," +
        #     str(float("{0:.2f}".format(float(self.twist.linear.x)))) + "," +
        #     str(float("{0:.2f}".format(float(self.twist.linear.y)))) + "," +
        #     str(float("{0:.2f}".format(float(self.twist.linear.z)))) + "," +
        #     str(float("{0:.2f}".format(float(self.position.x)))) + "," +
        #     str(float("{0:.2f}".format(float(self.position.y)))) + "," +
        #     str(float("{0:.2f}".format(float(self.position.z)))) +  '\n')
        # self.f.write(
        #     str(float("{0:.2f}".format(float(self.position.x)))) + "," +
        #     str(float("{0:.2f}".format(float(self.position.y)))) + "," +
        #     str(float("{0:.2f}".format(float(self.position.z)))) + '\n')
        # self.f.write(str(self.inte)+", ")
        #






if __name__ == "__main__":
    node = SelfRightPkg()
    node.run()
