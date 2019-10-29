import numpy as np
from math import sin, cos

class updateKin():
    def __init__(self):
        self.HT_LF_foot = np.zeros((3, 1), dtype=np.float)
        self.HT_RF_foot = np.zeros((3, 1), dtype=np.float)
        self.HT_LH_foot = np.zeros((3, 1), dtype=np.float)
        self.HT_RH_foot = np.zeros((3, 1), dtype=np.float)

    def updateHT_LF_foot(self,q):
        HT_LF_foot = np.zeros((3, 1), dtype=np.float)

        HT_LF_foot[0] = -0.380e0 * cos(q[2]) * sin(q[1]) * cos(q[0]) - 0.380e0 * sin(
            q[2]) * cos(q[1]) * cos(q[0]) - 0.360e0 * sin(q[1]) * cos(
            q[0]) + 0.4435000000e0 - 0.117e0 * sin(q[0])
        HT_LF_foot[1] = -0.380e0 * cos(q[2]) * sin(q[1]) * sin(q[0]) - 0.380e0 * sin(
            q[2]) * cos(q[1]) * sin(q[0]) - 0.360e0 * sin(q[1]) * sin(
            q[0]) + 0.9700000000e-1 + 0.117e0 * cos(q[0])
        HT_LF_foot[2] = -0.380e0 * cos(q[2]) * cos(q[1]) + 0.380e0 * sin(q[2]) * sin(
            q[1]) - 0.360e0 * cos(q[1])

        self.HT_LF_foot = HT_LF_foot
        return HT_LF_foot

    def updateHT_RF_foot(self, q):
        HT_RF_foot = np.zeros((3, 1), dtype=np.float)
        
        HT_RF_foot[0] = 0.380e0 * cos(q[1]) * sin(q[0]) * cos(q[2]) + 0.380e0 * sin(
            q[1]) * cos(q[0]) * cos(q[2]) + 0.360e0 * sin(q[0]) * cos(
            q[2]) + 0.4435000000e0 + 0.117e0 * sin(q[2])
        HT_RF_foot[1] = 0.380e0 * cos(q[1]) * sin(q[0]) * sin(q[2]) + 0.380e0 * sin(
            q[1]) * cos(q[0]) * sin(q[2]) + 0.360e0 * sin(q[0]) * sin(
            q[2]) - 0.9700000000e-1 - 0.117e0 * cos(q[2])
        HT_RF_foot[2] = -0.380e0 * cos(q[1]) * cos(q[0]) + 0.380e0 * sin(q[1]) * sin(
            q[0]) - 0.360e0 * cos(q[0])

        self.HT_RF_foot = HT_RF_foot
        return HT_RF_foot

    def updateHT_LH_foot(self, q):
        HT_LH_foot = np.zeros((3, 1), dtype=np.float)

        HT_LH_foot[0] = 0.380e0 * cos(q[1]) * sin(q[0]) * cos(q[2]) + 0.380e0 * sin(
            q[1]) * cos(q[0]) * cos(q[2]) + 0.360e0 * sin(q[0]) * cos(
            q[2]) - 0.4435000000e0 - 0.117e0 * sin(q[2])
        HT_LH_foot[1] = 0.380e0 * cos(q[1]) * sin(q[0]) * sin(q[2]) + 0.380e0 * sin(
            q[1]) * cos(q[0]) * sin(q[2]) + 0.360e0 * sin(q[0]) * sin(
            q[2]) + 0.9700000000e-1 + 0.117e0 * cos(q[2])
        HT_LH_foot[2] = 0.380e0 * cos(q[1]) * cos(q[0]) - 0.380e0 * sin(q[1]) * sin(
            q[0]) + 0.360e0 * cos(q[0])

        self.HT_LH_foot = HT_LH_foot
        return HT_LH_foot

    def updateHT_RH_foot(self, q):
        HT_RH_foot = np.zeros((3, 1), dtype=np.float)
        HT_RH_foot[0] = 0.380e0 * cos(q[2]) * sin(q[1]) * cos(q[0]) + 0.380e0 * sin(
            q[2]) * cos(q[1]) * cos(q[0]) + 0.360e0 * sin(q[1]) * cos(
            q[0]) - 0.4435000000e0 + 0.117e0 * sin(q[0])
        HT_RH_foot[1] = 0.380e0 * cos(q[2]) * sin(q[1]) * sin(q[0]) + 0.380e0 * sin(
            q[2]) * cos(q[1]) * sin(q[0]) + 0.360e0 * sin(q[1]) * sin(
            q[0]) - 0.9700000000e-1 - 0.117e0 * cos(q[0])
        HT_RH_foot[2] = -0.380e0 * cos(q[2]) * cos(q[1]) + 0.380e0 * sin(q[2]) * sin(
            q[1]) - 0.360e0 * cos(q[1])

        self.HT_RH_foot = HT_RH_foot
        return HT_RH_foot



