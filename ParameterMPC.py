"""
This is a class used to update the model and constraints of a predictive control problem of a self-right HyQ2Max robot.
Considering the states as:
    x[k/k] = [h[k/k] e[k/k]] in R 2*n x 2*n
where h[k/k] is the linear and angular momentum of the centroid, and e[k/k] the error of then in relationship with the
references momentous.

It is possible to write a nom linear system like shown bellow:
    x[k+1/k] = A*x[k/k] + ts*Agd[k/k]*qd[k/k] + ts*Ag[k/k]*qdd[k/k]
Rewrite  the system:
    x[k+1/k] = A*x[k/k] + B[k/k]*qdd[k/k]]'
where
    B[k/k] = ts*[Agd[k/k] Ag[k/k]] in R 2*n x p
    qb[k/k] = [qd[k/k] qdd[k/k]]' in R p x 1
The output is defined as:
    y[k/k] = C[k/k]*x[k/k] in R p X n

It is assumed that B[k/k] is constant for a small control horizon. So, the predicted model, for a control horizon M,
and prediction horizon N can be write as:

Y[k/k] = H*[qb[k/k] ... qd[k+M/k]' + Phi*x[k/k]

where:
    H = [ C*B     0     0  .... 0
          C*A*B   CB    0  .... 0
          C*A^2*B C*A*B CB .... 0
          ..................... 0
          C*A^N*B C*A^N-1*B ... CB]
and
    Phi = [CA CA^2 ... CA^N-1]
The dimensions of H depends of the M and N.

The system want to respect the friction cone, so, it is established a constraint:
    (C2*C1#*Wg)*[qb[k/k] ... qb[k+M/k]' <= [0 ... 0]'
This constraint depends of the pose of the robot.
The joints accelerations, speed and positions are also constrained:
    qdd_min <= qdd[k/k] <= qdd_max
    qd_min <= qd[k/k] + ts*qdd[k/k] <= qd_max
    q_min <= q[k/k] + ts*qd[k/k] + ts^2*qdd[k/k] <= q_max

Aureo Guilherme Dobrikopf,
Master student at Santa Catarina State University(UDESC)
aureogd@gmail.com

Version 0.0.1
25/10/2019
"""
import numpy as np
from scipy.linalg import block_diag
from DynModel import DynModel
from updateKin import updateKin
import os
import rospkg
from scipy import sparse
import time

DEFAULT_FRIC_COEF = 0.5

class ParametersMPC:

    def __init__(self, N, M, ts):
        """General variable"""
        self.N = N  # predict horizon
        self.M = M  # control horizon
        self.ts = ts  # sample time

        """Initialization of external class"""
        self.dynmodel = DynModel()  # Centroidal Momentum Matrix class
        self.kin = updateKin()  # Kinematics class
        self.rospack = rospkg.RosPack()

        """System"""
        self.n = 12  # number of states [h;x_d]
        self.n_a = self.n * 2
        self.p = 24  # number of control variables
        self.q = 12  # number of system output
        self.A = np.zeros((24, 24), dtype=np.float64)  # Dynamic Matrix, expanded
        #self.A[0:6, 0:6] = np.i
        # dentity(6)
        self.A[12:24, 0:12] = - self.ts * np.identity(12)
        self.A[12:24, 12:24] = np.identity(12)
        self.B = np.zeros((self.n_a, self.p), dtype=np.float64)  # input Matrix
        # self.C = np.concatenate((- self.ts * np.identity(self.q), (np.identity(self.q))), axis=1)  # Exit Matrix
        self.C = np.concatenate((np.identity(self.q), (np.zeros((self.q, self.q), dtype=np.float64))), axis=1)  # Exit Matrix
        self.x = np.zeros((self.n_a, 1), dtype=np.float)  # Augmented states vector: x = [h[k/k];x_d[k/k];
        # e_h[k/k];e_x[k/k]

        self.M_0 = np.zeros((self.n, self.n), dtype=np.float64)
        self.M_1 = np.zeros((self.n, self.n), dtype=np.float64)
        self.M_2 = np.zeros((self.n, self.n), dtype=np.float64)
        self.M_3 = np.zeros((self.n, self.n), dtype=np.float64)
        self.M_4 = np.zeros((self.n, self.n), dtype=np.float64)
        self.M_A_12 = np.zeros((self.n, self.n), dtype=np.float64)
        self.M_B_11 = np.zeros((self.n, self.q), dtype=np.float64)
        self.M_B_12 = np.zeros((self.n, self.q), dtype=np.float64)
        self.Ig_inv = np.zeros((self.n, self.q), dtype=np.float64)
        self.Igd_inv = np.zeros((self.n, self.q), dtype=np.float64)



        self.ref = None
        self.reference = None

        """Constraints"""
        self.A_Constraints = None  # Matrix of the constraints inequation  b_l<=Ax<=b_u
        self.A_aux = None
        self.b_Constraints = None
        self.b_l = np.zeros((self.M * (20 + 12 * 3), 1), dtype=np.float64)  # Matrix of the constraints b_l
        self.b_u = np.zeros((self.M * (20 + 12 * 3), 1), dtype=np.float64)  # Matrix of the constraints b_u
        self.b_CF = None
        self.C_fc = np.zeros((6, 12), dtype=np.float64)
        self.C_fp = None
        self.C1blk = None
        self.C_fp_blk = None
        self.Wg = None
        self.CF = None
        self.Ca = None
        self.Cs = None
        self.Cp = np.zeros((self.M * self.p / 2, self.M * self.p), dtype=np.float64)  # Position constraint matrix
        self.c_q_min = None
        self.c_q_max = None
        self.vetinf = np.inf * np.ones((20, 1))

        self.mass = 80.51000
        self.g = -9.81

        self.sumF = np.array([[0], [0], [0], [0], [0], [self.mass*self.g]], np.float64)

        """Controller Variable"""
        self.G = np.zeros((self.N * self.q, self.M * self.p))
        self.Phi = np.zeros((self.N * self.q, self.n_a))
        self.H = np.zeros((self.M * self.p, self.M * self.p))
        self.H_aux = np.zeros((self.M * self.p, self.M * self.p))
        self.F = np.zeros((1, self.M * self.p))

        """Auxiliary variables"""
        self.mtxSCR = np.zeros((3, 3), dtype=np.float)  # Auxiliary matrix to create the screw symmetric matrix

        '''Gain Matrices'''
        self.Py = None
        self.Pu = None
        # self.updategainmatrices(Q, R)

        self.lastmode = None
        self.params = None
        self.count = 0

        """ A_Constraints_lower constraint matrix """
        for i in range(self.M):  # Acceleration constraint matrix
            if i == 0:
                self.Ca = self.evalca()
            else:
                self.Ca = block_diag(self.Ca, self.evalca())

        for i in range(self.M):  # Speed constraint matrix
            if i == 0:
                self.Cs = self.evalcs()
            else:
                self.Cs = block_diag(self.Cs, self.evalcs())

        for i in range(self.M):
            for j in range(self.M):
                if j <= i:
                    self.Cp[i * self.p / 2:(i + 1) * self.p / 2, j * self.p:(j + 1) * self.p] = self.evalcp()

        self.A_Constraints_upper = np.vstack((self.Ca,
                                              self.Cs,
                                              self.Cp))

    def initmodel(self):
        zeros = np.zeros((self.M*20, self.M*24), dtype=np.float64)
        self.A_aux = np.concatenate((self.A_Constraints_upper, zeros), axis=0)
        self.A_Constraints = sparse.csc_matrix(self.A_aux)
        self.H = sparse.csc_matrix(self.H)

    def loaddata(self, mode):
        if mode == 'StandUp':
            self.params = np.load(os.path.join(self.rospack.get_path("self_right_ctrl"),
                                               "src", "constraints_standup.npz"))

        if mode == 'PrepAndRoll':
            self.params = np.load(os.path.join(self.rospack.get_path("self_right_ctrl"),
                                               "src", "constraints_sprepandroll.npz"))

        self.ref = self.params['ref_hx']

        self.updategainmatrices(self.params['Q'], self.params['R'])

        self.b_u = self.staticconstrain([self.params['qdd_max'], self.params['qd_max']])
        self.b_l = self.staticconstrain([self.params['qdd_min'], self.params['qd_min']])



        self.c_q_min = self.params['q_min'].reshape(12, 1)
        self.c_q_max = self.params['q_max'].reshape(12, 1)

    def updategainmatrices(self, Q, R):
        for i in range(self.N):  # States weighting matrix
            if i == 0:
                self.Py = Q
            else:
                self.Py = block_diag(self.Py, Q)

        self.Py = self.Py

        for i in range(self.M):  # Input weighting matrix
            if i == 0:
                self.Pu = R
            else:
                self.Pu = block_diag(self.Pu, R)

        self.Pu = self.Pu

        print self.Pu, self.Py

    """Evaluate the acceleration constraint matrix block"""

    def evalca(self):
        auxca = np.concatenate((np.zeros((self.p / 2, self.p / 2), dtype=np.float64),
                                np.identity(self.p / 2, dtype=np.float64)), axis=1)
        return auxca

    """Evaluate the speed constraint matrix block"""

    def evalcs(self):
        auxcs = np.concatenate((np.identity(self.p / 2, dtype=np.float64),
                                self.ts * np.identity(self.p / 2, dtype=np.float64)), axis=1)
        return auxcs

    """Evaluate the position constraint matrix block"""

    def evalcp(self):
        auxcp = np.concatenate((self.ts * np.identity(self.p / 2, dtype=np.float64),
                                self.ts**2 * np.identity(self.p / 2, dtype=np.float64)), axis=1)
        return auxcp

    def scr(self, q):  # Function used to evaluate the screw symmetric matrix
        q.reshape(3, 1)
        return np.array([[0, -q[2, 0], q[1, 0]], [q[2, 0], 0, -q[0, 0]], [-q[1, 0], q[0, 0], 0]])

    def staticconstrain(self, constraints_list):

        flag_first = True
        for i in constraints_list:
            for j in range(self.M):
                if flag_first == True:
                    flag_first = False
                    b_Constraints = self.ts * i.reshape(self.p / 2, 1)
                else:
                    b_Constraints = np.vstack((b_Constraints, self.ts * i.reshape(self.p / 2, 1)))
        b_Constraints = np.vstack((b_Constraints, np.zeros((self.M * self.p / 2, 1), dtype=np.float)))
        b_Constraints = np.vstack((b_Constraints, np.zeros((self.M * 20, 1), dtype=np.float)))
        return b_Constraints

    def updatedynconstraint(self, con_list, con_p, q, qd, x, fric_coef=None):
        if fric_coef is None:
            self.C_fp = np.array([[1, 0, -DEFAULT_FRIC_COEF],
                                [-1, 0, -DEFAULT_FRIC_COEF],
                                [0, 1, -DEFAULT_FRIC_COEF],
                                [0, -1, -DEFAULT_FRIC_COEF],
                                [0, 0, -1]])
        else:
            self.C_fp = np.array([[1, 0, -fric_coef],
                                [-1, 0, -fric_coef],
                                [0, 1, -fric_coef],
                                [0, -1, -fric_coef],
                                [0, 0, -1]])

        for i in range(4):
                if i == 0:
                    self.C_fp_blk = self.C_fp
                else:
                    self.C_fp_blk = block_diag(self.C_fp_blk, self.C_fp)

        self.C_fc.fill(0)

        for i in range(4):
            if con_list[i] == True:
                self.C_fc[:, i*3:(i+1)*3 ] = np.concatenate((self.scr(con_p[i*3:(i+1)*3]), np.identity(3)), axis=0)

        C_in_11 = self.dynmodel.Agd_inf + np.matmul(self.M_2, self.dynmodel.Agd_inf)
        C_in_12 = self.dynmodel.Ag_inf + np.matmul(self.M_2, self.dynmodel.Ag_inf)
        C_st_11 = np.matmul(self.dynmodel.Ag_sup, self.M_1)
        C_st_12 = self.dynmodel.Agd_sup +  np.matmul(self.M_2, self.dynmodel.Agd_sup)

        C_in = np.concatenate((C_in_11, C_in_12), axis= 1)
        C_st = np.concatenate((C_st_11, C_st_12), axis=1)

        C_com = np.matmul(self.C_fp_blk, np.linalg.pinv(self.C_fc))
        C_A_f = np.matmul(C_com, C_in)
        C_b_f = np.matmul(C_com, (-np.matmul(C_st, x)+ self.sumF))
        # C_b_f = np.matmul(C_com, (-np.matmul(C_st, x)))

        for i in range(self.M):
            if i == 0:
                self.CF = C_A_f
            else:
                self.CF = block_diag(self.CF, C_A_f)

        self.A_aux = np.concatenate((self.A_Constraints_upper, self.CF), axis=0)

        # self.A_Constraints = np.concatenate((self.A_Constraints_upper, np.zeros((self.M*20, self.M*24), dtype=np.float64)), axis=0)
        b_p_max = np.add(self.c_q_max, -q)
        b_p_min = np.add(self.c_q_min, -q)

        for i in range(self.M):
            a = i * 12 + 12 * self.M * 2
            b = (i + 1) * 12 + 12 * self.M * 2
            self.b_u[a:b] = b_p_max
            self.b_l[a:b] = b_p_min

        for i in range(self.M):
            a = i * 20 + 12 * self.M * 3
            b = (i + 1) * 20 + 12 * self.M * 3
            # print i, a, b, C_b_f.shape, self.b_u.shape
            self.b_u[a:b] = C_b_f
            self.b_u[a:b] = self.vetinf
            self.b_l[a:b] = -self.vetinf

        self.b_l.fill(-np.inf)
        self.b_u.fill(np.inf)

        #print self.b_u, self.b_l

    def upadateSystemmatrix(self, q, qd):

        #self.dynmodel.updateall(q, qd[6:18])
        self.dynmodel.updateall(q, qd[6:18])

        self.Ig_inv = np.linalg.inv(self.dynmodel.Ig)
        self.Igd_inv = -np.matmul(np.matmul(self.Ig_inv, self.dynmodel.Igd), self.Ig_inv)

        self.M_0 = np.linalg.inv(np.identity(6) - np.matmul(self.Ig_inv, self.dynmodel.Ag_sup))
        self.M_1 = np.matmul(self.M_0, self.Igd_inv)
        self.M_2 = np.matmul(np.matmul(self.dynmodel.Ag_sup, self.M_0), self.Ig_inv)
        self.M_3 = np.matmul(self.M_0, self.Ig_inv)

        self.A[0:6, 0:6] = np.identity(6) + self.ts * (np.matmul(self.dynmodel.Ag_sup, self.M_1))
        self.A[0:6, 6:12] = self.ts * (self.dynmodel.Agd_sup + np.matmul(self.M_2, self.dynmodel.Agd_sup)) #
        self.A[6:12, 0:6] = self.Ig_inv + self.ts * self.M_1
        self.A[6:12, 6:12] = self.ts * np.matmul(self.M_3, self.dynmodel.Agd_sup)

        self.B[0:6, 0:12] = self.ts * (self.dynmodel.Agd_inf + np.matmul(self.M_2, self.dynmodel.Agd_inf))
        self.B[0:6, 12:24] = self.ts * (self.dynmodel.Ag_inf + np.matmul(self.M_2, self.dynmodel.Ag_inf))
        self.B[6:12, 0:12] = self.ts * np.matmul(self.M_3, self.dynmodel.Agd_inf)
        self.B[6:12, 12:24] = self.ts * np.matmul(self.M_3, self.dynmodel.Ag_inf)

    def updatemodel(self, q, qd, mode, con_list, con_p, sum_t):
        if mode != self.lastmode:
            self.loaddata(mode)
            self.lastmode = mode
            self.count = 0

        #self.sumF[0:3,:] = sum_t
        # update system
        self.upadateSystemmatrix(q, qd)

        self.x[0:6] = np.matmul(self.dynmodel.Ag_sup, qd[0:6]) + np.matmul(self.dynmodel.Ag_inf, qd[6:18])
        self.x[6:12] = qd[0:6]

        self.x[12:24] = self.ref[:, self.count].reshape(12, 1) - self.x[0:12, :]

        print self.x

        # if self.count == 0:
        #     self.x.fill(0)

        #print self.x[0:6], self.count

        """G = [CB ... 0; CAB CB ... 0]"""
        for i in range(self.N):  # generate the matrix G
            for j in range(self.M):
                c = j + 1
                if c <= i + 1:
                    self.G[i * self.q:self.q * (i + 1), self.p * j:self.p * (j + 1)] = \
                        np.matmul(np.matmul(self.C, (np.linalg.matrix_power(self.A, (i + 1 - c)))), self.B)

        """H = 2*(G'*Py*G+Pu)"""
        self.H = 1 * (np.matmul(np.matmul(np.transpose(self.G), self.Py), self.G) + self.Pu)
        self.H_aux = self.H

        """Phi Matrix: [CA, CA^2, ..., CA^N]"""
        for i in range(self.N):
            self.Phi[i * self.q:self.q * (i + 1), :] = np.matmul(self.C, np.linalg.matrix_power(self.A, i + 1))


        """Vector of the reference in a predict horizon"""
        ref_in_horizon = None
        for i in range(self.N):
            if i == 0:
                ref_in_horizon = self.ref[:, self.count]
                ref_in_horizon = np.expand_dims(ref_in_horizon, axis=1)
            else:
                ref_in_horizon = np.vstack((ref_in_horizon, np.expand_dims(self.ref[:, i + self.count], axis=1)))
        self.reference = ref_in_horizon

        #print ref_in_horizon

        self.count += 1
        """F = 2*(Phi*x-Ref)'*Py*G"""
        self.F = 1 * np.matmul(np.matmul(np.transpose(np.matmul(self.Phi, self.x)- ref_in_horizon), self.Py), self.G)

        """Constraints: the constraints depends from mode of the posture of the robot """

        self.updatedynconstraint(con_list, con_p, q, qd, self.x[0:12])
        self.A_Constraints = sparse.csc_matrix(self.A_aux)
        self.H = sparse.csc_matrix(self.H)

