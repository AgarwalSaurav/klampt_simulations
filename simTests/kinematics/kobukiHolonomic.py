#!/usr/bin/python
## The class contains wrapper functions to set and get configuration of 3DoF holonomic robot
## The first three elements of the configuration are (x, y, z)
## The next three are zyx Euler angles
## The function getTransform is for getting the rotation and the translation of the current position of the robot

from klampt import *
from klampt import vis
from klampt.vis.glcommon import GLWidgetPlugin
from klampt.math import so3
import mathUtils
class kobukiHolonomic(object):
    def __init__ (self, robot, vis=None):
        self.robot = robot
        self.vis = vis
        rotMat = so3.identity()
        pt = [0, 0, 0]
        if self.vis is not None:
            self.vis.add("Robot",[rotMat, pt])
            self.vis.setAttribute("Robot", "size", 32)
            self.vis.edit("Robot")

    def getConfig(self):
        q = self.robot.getConfig()
        return [q[0], q[1], q[3]]

    def setConfig(self, qC):
        q = self.robot.getConfig()
        q[0] = qC[0]
        q[1] = qC[1]
        q[3] = qC[2]
        self.robot.setConfig(q)
        if self.vis is not None:
            trans = self.getTransform()
            rotMat = trans[0]
            pt = trans[1]
            self.vis.add("Robot",[rotMat, pt], keepAppearance=True)
        
    def getTransform(self):
        q = self.robot.getConfig()
        theta = [q[3], q[4], q[5]]
        rotMat = mathUtils.euler_zyx_mat(theta)
        return [rotMat, [q[0], q[1], q[2]]]

    def setAltitude(self, alt):
        q = self.robot.getConfig()
        q[2] = alt
        self.robot.setConfig(q)
        if self.vis is not None:
            trans = self.getTransform()
            rotMat = trans[0]
            pt = trans[1]
            self.vis.add("Robot",[rotMat, pt], keepAppearance=True)
