'''
This is a class used to update the model and constraints of a predictive control problem of a self-right HyQ2Max robot.

Aureo Guilherme Dobrikopf,
Master student at Santa Catarina State University(UDESC)
aureogd@gmail.com

Version 0.0.1
25/10/2019
'''
import numpy as np
from scipy.linalg import block_diag
from CentroidalMomentumMatrix import CentroidalMomentumMatrix
from updateKin import updateKin


class ParametersMPC:

    def __init__(self, N, M, ts, qmax, qmin, qdmax, qdmin, Q, R, fricCoef):
        '''General variable'''
        self.N = N #predict horizon
        self.M = M #control horizon
        self.ts = ts #sample time

        '''System'''
        self.n = 6  # number of states
        self.A = np.concatenate((np.concatenate((np.identity(self.n), np.zeros((self.n,self.n))), axis=1),
                                 np.concatenate((ts*np.identity(self.n), np.identity((self.n))), axis=1)), axis=0) #Dynamic Matrix
        self.B = np.zeros((6,36), dtype=np.float64) #imput Matrix
        #self.B = np.zeros((6, 18), dtype=np.float64)  # imput Matrix
        self.C = np.concatenate((np.identity(self.n),np.zeros((self.n, self.n))), axis=1) #Exit Matrix
        self.x = np.zeros((1,12), dtype=np.float) #Aumented states vector: x = [h[k/k];e[k/k]]
        self.cmm = CentroidalMomentumMatrix() #Initialization of the Centroidal Momentum Matrix
        self.cmmMtx = np.zeros((6, 18), dtype=np.float) #Ag matrix
        self.cmmdMtx = np.zeros((6, 18), dtype=np.float) #Ag dot matrix
        self.kin = updateKin()

        '''Constraints'''
        self.C1 = ''
        self.C2 = np.array([[1, 0, -fricCoef],
                            [-1, 0, -fricCoef],
                            [0, 1, -fricCoef],
                            [0, -1, -fricCoef]])
        self.C2blk = ''
        self.Wg = ''
        self.qmax = qmax
        self.qmin = qmin
        self.qdmax = qdmax
        self.qdmin = qdmin
        self.qddblk = np.concatenate((np.identity(18, dtype=np.float64),
                                      np.zeros((18, 18), dtype=np.float64)), axis=1)
        self.qdblk = np.concatenate((np.identity(18, dtype=np.float64)*self.ts,
                                     np.identity(18, dtype=np.float64)), axis=1)
        self.qlk = np.concatenate((np.zeros((18, 18), dtype=np.float64),
                                   np.identity(18, dtype=np.float64)*self.ts), axis=1)

        '''Controller Variable'''
        self.G = np.zeros((self.N * self.n * 2, self.M * self.n * 2))
        self.Phi = np.zeros((self. N * self.n, self.n * 2))
        self.H = np.zeros((self.N * self.n * 2), 36 * self.M)
        self.F = ''

        '''Auxiliary variables'''
        self.mtxSCR = np.zeros((3, 3), dtype=np.float) #Auxiliary matrix to create the screw symmetric matrix

        '''Constants Matrices'''
        for i in range(self.N):
            self.Phi[i * self.n:self.n * (i + 1), :] = self.C.dot(np.linalg.matrix_power(self.A, i))

        for i in range(self.N): #States ponderation matrix
            if i == 0:
                self.Py = Q
            else:
                self.Py = block_diag(self.Py, Q)

        for i in range(self.M): #Imput ponderation matrix
            if i == 0:
                self.Pu = R
            else:
                self.Pu = block_diag(self.Pu, R)



    def scr(self, q): #Function used to evaluate the screw symmetric matrix
        self.mtxSCR[0][1] = -q[2]
        self.mtxSCR[0][2] = q[1]
        self.mtxSCR[1][0] = q[2]
        self.mtxSCR[1][2] = -q[0]
        self.mtxSCR[2][0] = q[1]
        self.mtxSCR[2][1] = q[0]

    def updatemodel(self, q, qd, ref, mode):
        self.cmmMtx = self.cmm.updateCCM(q) #update the mass matrix Ag
        self.x[0:self.n+1] = self.cmmMtx.dot(qd) #update the momentum values
        self.x[self.n+1:] = self.x[0:self.n+1]-ref[0:self.n+1] #Evaluate the error of the desired momentum

        self.cmmdMtx = self.cmm.updateCMMd(q, qd) #upadate Ag dot
        self.B = np.concatenate((self.cmmdMtx, self.cmmMtx), axis=1)*self.ts #update matrix B values
        #self.B = self.ts*(self.cmmdMtx * self.ts+self.cmmMtx) #update matrix B values

        for i in range(self.N): #generete the matrix G
            l = i + 1
            for j in range(self.M):
                c = j + 1
                if c <= l:
                    self.G[i * self.n:self.n*(i + 1), self.n * j:self.n * (j + 1)] =\
                        (self.C.dot(np.linalg.matrix_power(self.A, (l - c)))).dot(self.B)
        '''H = 2*(G'*Py*G+Pu)'''
        self.H = (2*(np.transpose(self.G).dot(self.Py)).dot(self.G) + self.Pu)
        '''F = (Phi*x-Ref)'*Py*G'''
        self.F = 2 * ((np.transpose(self.Phi.dot(self.x)-ref)).dot(self.Py)).dot(self.G)

        '''Constraints: the constraints depends from mode of the posture of the robot '''
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

        rest1 = (self.C2blk.dot(np.linalg.pinv(self.C1))).dot(self.Wq)







m = ParametersMpC(5, 3, 0.04)


