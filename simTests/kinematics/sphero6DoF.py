import mathUtils
class sphero6DoF(object):
    def __init__ (self, robot):
        self.robot = robot

    def getConfig(self):
        q = self.robot.getConfig()
        return [q[0], q[1], q[2], q[3], q[4], q[5]]

    def setConfig(self, qC):
        q = self.robot.getConfig()
        q[0] = qC[0]
        q[1] = qC[1]
        q[2] = qC[2]
        q[3] = qC[3]
        q[4] = qC[4]
        q[5] = qC[5]
        self.robot.setConfig(q)
        
    def getTransform(self):
        q = self.robot.getConfig()
        theta = [q[3], q[4], q[5]]
        rotMat = mathUtils.euler_zyx_mat(theta)
        return [rotMat, [q[0], q[1], q[2]]]
