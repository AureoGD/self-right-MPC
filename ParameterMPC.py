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
    (C2*C1†*Wg)*[qb[k/k] ... qb[k+M/k]' <= [0 ... 0]'
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
from CentroidalMomentumMatrix import CentroidalMomentumMatrix
from updateKin import updateKin

DEFAULT_FRIC_COEF = 1


class ParametersMPC:

    def __init__(self, N, M, R, Q, ts):
        """General variable"""
        self.N = N  # predict horizon
        self.M = M  # control horizon
        self.ts = ts  # sample time

        """Initialization of external class"""
        self.cmm = CentroidalMomentumMatrix()  # Centroidal Momentum Matrix class
        self.kin = updateKin()  # Kinematics class

        """System"""
        self.n = 6  # number of states
        self.A = np.concatenate((np.concatenate((np.identity(self.n), np.zeros((self.n, self.n))), axis=1),
                                 np.concatenate((ts*np.identity(self.n), np.identity(self.n)), axis=1)), axis=0)  # Dynamic Matrix
        self.B = np.zeros((6, 36), dtype=np.float64)  # input Matrix
        self.C = np.concatenate((np.identity(self.n), np.zeros((self.n, self.n))), axis=1)  # Exit Matrix
        self.x = np.zeros((1, 12), dtype=np.float)  # Augmented states vector: x = [h[k/k];e[k/k]]
        self.cmm_Mtx = np.zeros((6, 18), dtype=np.float)  # Ag matrix
        self.cmm_d_Mtx = np.zeros((6, 18), dtype=np.float)  # Ag dot matrix

        """Constraints"""
        self.A_Constraints = ''  # Matrix of the constraints inequation  Ax<=b
        self.b_Constraints = ''  # Matrix of the constraints inequation  Ax<=b
        self.C1 = ''
        self.C2 = ''
        self.C2blk = ''
        self.Wg = ''
        self.CF = ''
        self.Ca = ''
        self.Cs = ''
        self.Cp = np.zeros((self.M * 18, self.M * 36), dtype=np.float64)  # Position constraint matrix

        """Controller Variable"""
        self.G = np.zeros((self.N * self.n * 2, self.M * self.n * 2))
        self.Phi = np.zeros((self. N * self.n, self.n * 2))
        self.H = np.zeros((self.N * self.n * 2), 36 * self.M)
        self.F = ''

        """Auxiliary variables"""
        self.mtxSCR = np.zeros((3, 3), dtype=np.float)  # Auxiliary matrix to create the screw symmetric matrix

        """Constants Matrices"""
        for i in range(self.N):
            self.Phi[i * self.n:self.n * (i + 1), :] = self.C.dot(np.linalg.matrix_power(self.A, i+1))

        '''Gain Matrices'''
        self.Py = ''
        self.Pu = ''
        self.updategainmatrices(Q, R)

    def updategainmatrices(self, Q, R):
        for i in range(self.N):  # States weighting matrix
            if i == 0:
                self.Py = Q
            else:
                self.Py = block_diag(self.Py, Q)

        for i in range(self.M):  # Input weighting matrix
            if i == 0:
                self.Pu = R
            else:
                self.Pu = block_diag(self.Pu, R)

    """Evaluate the acceleration constraint matrix block"""
    def evalca(self):
        auxca = np.concatenate((np.zeros((6, 36), dtype=np.float64),
                                np.concatenate((np.zeros((12, 24), dtype=np.float64),
                                                np.identity(12, dtype=np.float64)), axis=1)),
                               axis=0)
        return auxca

    """Evaluate the speed constraint matrix block"""
    def evalcs(self):
        auxcs = np.concatenate((np.zeros((6, 36), dtype=np.float64),
                                np.concatenate((np.zeros((12, 6), dtype=np.float64),
                                                np.identity(12, dtype=np.float64),
                                                np.zeros((12, 6), dtype=np.float64),
                                                self.ts*np.identity(12, dtype=np.float64)), axis=1)),
                               axis=0)
        return auxcs

    """Evaluate the position constraint matrix block"""
    def evalcp(self):
        auxcp = np.concatenate((np.zeros((6, 36), dtype=np.float64),
                                np.concatenate((np.zeros((12, 6), dtype=np.float64),
                                                self.ts * np.identity(12, dtype=np.float64),
                                                np.zeros((12, 6), dtype=np.float64),
                                                (self.ts ** 2) * np.identity(12, dtype=np.float64)), axis=1)),
                               axis=0)
        return auxcp

    def scr(self, q):  # Function used to evaluate the screw symmetric matrix
        self.mtxSCR[0][1] = -q[2]
        self.mtxSCR[0][2] = q[1]
        self.mtxSCR[1][0] = q[2]
        self.mtxSCR[1][2] = -q[0]
        self.mtxSCR[2][0] = q[1]
        self.mtxSCR[2][1] = q[0]

    def updateconstraint(self, Mode, q, fric_coef=None):
        if fric_coef is None:
            self.C2 = np.array([[1, 0, -DEFAULT_FRIC_COEF],
                                [-1, 0, -DEFAULT_FRIC_COEF],
                                [0, 1, -DEFAULT_FRIC_COEF],
                                [0, -1, -DEFAULT_FRIC_COEF]])
        else:
            self.C2 = np.array([[1, 0, -fric_coef],
                                [-1, 0, -fric_coef],
                                [0, 1, -fric_coef],
                                [0, -1, -fric_coef]])

        if Mode is 'StandUp':
            """Need to update the values"""
            q_max = np.array([[2*np.pi/9], [3*np.pi/4], [-2.85],
                              [-0.05], [1.57], [-2.85],
                              [-0.05], [-1.57], [2.85],
                              [-0.05], [-1.57], [2.85]])
            q_min = np.array([[2*np.pi/9], [3*np.pi/4], [-2.85],
                              [-0.05], [1.57], [-2.85],
                              [-0.05], [-1.57], [2.85],
                              [-0.05], [-1.57], [2.85]])
            qd_max = np.array([[2*np.pi/9], [3*np.pi/4], [-2.85],
                              [-0.05], [1.57], [-2.85],
                              [-0.05], [-1.57], [2.85],
                              [-0.05], [-1.57], [2.85]])
            qd_min = np.array([[2*np.pi/9], [3*np.pi/4], [-2.85],
                              [-0.05], [1.57], [-2.85],
                              [-0.05], [-1.57], [2.85],
                              [-0.05], [-1.57], [2.85]])
            qdd_max = np.array([[2*np.pi/9], [3*np.pi/4], [-2.85],
                              [-0.05], [1.57], [-2.85],
                              [-0.05], [-1.57], [2.85],
                              [-0.05], [-1.57], [2.85]])
            qdd_min = np.array([[2*np.pi/9], [3*np.pi/4], [-2.85],
                              [-0.05], [1.57], [-2.85],
                              [-0.05], [-1.57], [2.85],
                              [-0.05], [-1.57], [2.85]])

            for i in range(self.N):   # Acceleration constraint matrix
                if i == 0:
                    self.Ca = self.evalca()
                else:
                    self.Ca = block_diag(self.Ca, self.evalca())

            for i in range(self.N):  # Speed constraint matrix
                if i == 0:
                    self.Cs = self.evalcs()
                else:
                    self.Cs = block_diag(self.Cs, self.evalcs())

            for i in range(self.M):
                for j in range(self.M):
                    if i >= j:
                        self.Cp[i * 18:(i + 1) * 18, j * 36:(j + 1) * 36] = self.evalcp()

            self.C1 = np.concatenate((np.concatenate((self.scr(self.kin.updateHT_LF_foot(q[0:3])),
                                                      self.scr(self.kin.updateHT_RF_foot(q[3:6])),
                                                      self.scr(self.kin.updateHT_LH_foot(q[6:9])),
                                                      self.scr(self.kin.updateHT_RH_foot(q[9:12]))), axis=1),
                                      np.ones((3, 12), dtype=np.float64)), axis=0)
            for i in range(self.M):
                if i == 0:
                    self.Wg = self.B
                else:
                    self.Wg = block_diag(self.Wg, self.B)

            for i in range(4):
                if i == 0:
                    self.C2blk = self.C2
                else:
                    self.C2blk = block_diag(self.C2blk, self.C2)

            self.CF = (self.C2blk.dot(np.linalg.pinv(self.C1))).dot(self.Wg)
            self.A_Constraints = np.vstack((self.CF,
                                            self.Ca,
                                            -self.Ca,
                                            self.Cs,
                                            -self.Cs,
                                            self.Cp,
                                            -self.Cp))

            conTypes = [qdd_max, -qdd_min, qd_max, -qd_min, q_max-q[6:18], -q_min+q[6:18]]
            self.b_Constraints = np.zeros((16, 1), dtype=np.float64)

            for i in conTypes:
                for j in range(self.M):
                    self.b_Constraints = np.vstack(self.b_Constraints, i)

    def updatemodel(self, q, qd, ref, mode):
        self.cmm_Mtx = self.cmm.updateCCM(q)  # update the mass matrix Ag
        self.x[0:self.n+1] = self.cmm_Mtx.dot(qd)  # update the momentum values
        self.x[self.n+1:] = self.x[0:self.n+1]-ref[0:self.n+1]  # Evaluate the error of the desired momentum

        self.cmm_d_Mtx = self.cmm.updateCMMd(q, qd)  # update Ag dot
        self.B = np.concatenate((self.cmm_d_Mtx, self.cmm_Mtx), axis=1)*self.ts  # update matrix B values

        for i in range(self.N):  # generate the matrix G
            l = i + 1
            for j in range(self.M):
                c = j + 1
                if c <= l:
                    self.G[i * self.n:self.n*(i + 1), self.n * j:self.n * (j + 1)] =\
                        (self.C.dot(np.linalg.matrix_power(self.A, (l - c)))).dot(self.B)
        """H = 2*(G'*Py*G+Pu)"""
        self.H = 2*((np.transpose(self.G).dot(self.Py)).dot(self.G) + self.Pu)
        """F = 2*(Phi*x-Ref)'*Py*G"""
        self.F = 2*((np.transpose(self.Phi.dot(self.x)-ref)).dot(self.Py)).dot(self.G)

        """Constraints: the constraints depends from mode of the posture of the robot """
        mode = 'StandUp'
        self.updateconstraint(mode, q)
