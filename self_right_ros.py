#!/usr/bin/env python
"""

Aureo Guilherme Dobrikopf,
Master student at Santa Catarina State University(UDESC)
aureogd@gmail.com

Version 0.0.1
31/10/2019
"""

import rospy
import ParameterMPC

class SelfRightPkg():

    def __init__(self):
        rospy.init_mode('SelfRight')

        self.pose = None
        self.q = None
        self.qd = None

        self.MPCparams(6, 3, 0.04)

    def run(self):

        while not rospy.is_shytdown():




if __name__ == "__main__":
    node = SelfRightPkg()
    node.run()