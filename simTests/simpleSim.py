#!/usr/bin/python
## A simple demo wherein a sphere, which can move in R^3, is provided a circular trajectory. The sphere first moves to the end of the circle in the x-axis following a cubic trajectory. The sphere then, after pausing for sometime, moves in a circular path with constant angular velocity.
## The green sphere shows the commanded configuration of the robot.
## The gray sphere shows the actual configuration of the robot.
## The code plots the error at the end of the simulation.
import sys
from klampt import *
from klampt import vis
from klampt.vis import GLSimulationPlugin
import math
import matplotlib.pyplot as plt

class MyGLViewer(GLSimulationPlugin):

    ## The following variables are to be set
    startTime = 5.0 # Time to wait before giving commands (for settling of system such as ball falling)
    pauseTime = 5.0 # Pause time between two events
    maxVel = 0.2 # Max linear velocity for robot in any directin
    cirRad = 1.0 # Radius of circular trajectory
    nRev = 2.0 # Number of revolutions
    
    ## The following variable are computed in the control_loop function
    pi = [0.0, 0.0] # Initial positon
    pf = [0.0, 0.0] # Final position
    
    tfLin = 0.0 # Time taken to comoplete straight line cubic trajectory
    linearEndTime = 0.0 # End time for straight line trajectory
    circStartTime = 0.0 # Start time for circular trajectory
    circEndTime = 0.0 # End time for circular trajectory
    omegaVal = 0.0 # Angular velocity for circular trajectory

    q_record = [] # For recording the given config
    qSensed_record = [] # For recording the sensed config
    error_record = [] # For recording the error
    t_record =[] # FOr recording time

    plot_flag = 0

    def __init__(self,files):
        #create a world from the given files
        world = WorldModel()
        for fn in files:
            res = world.readFile(fn)
            if not res:
                raise RuntimeError("Unable to load model "+fn)
        #initialize the simulation
        GLSimulationPlugin.__init__(self,world)
        sim = self.sim
        #put custom action hooks here
        self.add_action(self.plot_fn,'Plot the recorded data','p')



    ## Compute the scaling parameter for cubic trajectory
    def cubicTrajec(self, t):
        tf = self.tfLin
        trVal = [t**2 / tf**2 * (3.0 - 2.0 * t / tf), (6.0 * t / tf**2) * (1.0 - t/tf)]
        return trVal

    def control_loop(self):
        #Put your control handler here
        sim = self.sim

        #if sim.getTime() >= 2.0 and sim.getTime()-self.dt < 2.0:
        #    q=sim.controller(0).getCommandedConfig()
        #    q[0]-=1.0
        #    q[1]-=1.0
        #    sim.controller(0).setMilestone(q)

        robot = sim.world.robot(0) 
        
        q = robot.getConfig()
        qSensed = sim.controller(0).getSensedConfig()

        q_error = [q[0] - qSensed[0], q[1] - qSensed[1], q[2] - qSensed[2]]
        error_norm =  (math.sqrt(q_error[0] ** 2) + q_error[1] ** 2 + q_error[2] ** 2)

        # Setup the constants
        if sim.getTime() < self.startTime:
            self.pi = [q[0], q[1]]
            self.pf[0] = self.cirRad
            self.pf[1] = self.pi[1]
            self.tfLin = (3.0 * (self.pf[0] - self.pi[0]))/(2.0 * self.maxVel)
            self.omegaVal = self.maxVel/self.cirRad
            self.circEndTime = 2.0 * 3.14 * self.nRev / self.omegaVal + self.linearEndTime
            self.linearEndTime = self.tfLin + self.startTime
            self.circStartTime = self.linearEndTime + self.pauseTime
            dq = [0, 0, 0]
            sim.controller(0).setPIDCommand(q,dq)

        if sim.getTime() > self.startTime and sim.getTime() < self.circEndTime:
            self.q_record += [q]
            self.qSensed_record += [qSensed]
            self.error_record.append(error_norm) 
            self.t_record.append(sim.getTime())


        ## Move in a straight path with cubic trajectory
        ## Although the path is just along the x-axis, a 2D formulation is done for the sake of completion
        if sim.getTime() > self.startTime  and sim.getTime() < self.linearEndTime:
            t = sim.getTime() - self.startTime
            trVal = self.cubicTrajec(t)
            q[0] = self.pi[0] + (self.pf[0] - self.pi[0]) * trVal[0]
            q[1] = self.pi[1] + (self.pf[1] - self.pi[1]) * trVal[0]

            ## dq is the velocity
            dq = [(self.pf[0] - self.pi[0]) * trVal[1], (self.pf[1] - self.pi[1]) * trVal[1], 0]

            ## Send position and velocity to PID controller
            sim.controller(0).setPIDCommand(q,dq)

        ## Pause after reaching the start position of the circular path
        if sim.getTime() > self.linearEndTime and sim.getTime() < self.circStartTime:
            dq = [0, 0, 0]
            sim.controller(0).setPIDCommand(q,dq)

        ## Circular path for constant angular velocity
        if sim.getTime() > self.linearEndTime and sim.getTime() < self.circEndTime:
            tht = (sim.getTime() - self.linearEndTime) * self.omegaVal
            q[0] = self.pi[0] + self.cirRad * math.cos(tht)
            q[1] = self.pi[1] + self.cirRad * math.sin(tht)
            dq = [-self.maxVel * math.sin(tht), self.maxVel * math.cos(tht), 0]

            sim.controller(0).setPIDCommand(q,dq)

        if sim.getTime() > self.circEndTime and self.plot_flag == 0:
            self.plot_flag = 1
            self.plot_fn()

        pass


    def plot_fn(self):
        plt.figure(figsize=(12, 14))    
  
        # Remove the plot frame lines. They are unnecessary chartjunk.    
        fig = plt.figure(1)
        lines = plt.plot(self.t_record, self.error_record)
        ax = plt.gca()    
        ax.spines["top"].set_visible(False)    
        #ax.spines["bottom"].set_visible(False)    
        ax.spines["right"].set_visible(False)    
        #ax.spines["left"].set_visible(False)    
        ax.get_xaxis().tick_bottom()    
        ax.get_yaxis().tick_left()  
        plt.xlabel('Time (s)')
        plt.ylabel('Error (m)')
        plt.title('Error as norm of difference in sensed and commanded configuration')
        ax.patch.set_facecolor([1, 1, 1])
        fig.patch.set_facecolor([1, 1, 1])
        plt.setp(lines, color=(0, 0.44, 0.74))

        plt.show()
        q_x = [x[0] for x in self.q_record]
        qSensed_x =[x[0] for x in self.qSensed_record]
        q_y = [x[1] for x in self.q_record]
        qSensed_y =[x[1] for x in self.qSensed_record]
        fig1 = plt.figure(2)
        plt.figure(num=2, figsize=(14, 14))    
        plotPath = plt.plot(q_x, q_y, color=(0, 0.44, 0.74))
        plt.plot(qSensed_x, qSensed_y, color=(0.85, 0.32, 0.1))
        plt.legend( ('Commanded', 'Sensed') )
        ax = plt.gca()    
        ax.set_xlim([-1.5, 1.5])
        ax.set_ylim([-1.5, 1.5])

        plt.gca().set_aspect('equal', adjustable='box')
        ax.spines["top"].set_visible(False)    
        #ax.spines["bottom"].set_visible(False)    
        ax.spines["right"].set_visible(False)    
        #ax.spines["left"].set_visible(False)    
        ax.get_xaxis().tick_bottom()    
        ax.get_yaxis().tick_left()  
        plt.xlabel('x-axis')
        plt.ylabel('y-axis')
        plt.title('The commanded and sensed path of the robot')
        ax.patch.set_facecolor([1, 1, 1])
        fig1.patch.set_facecolor([1, 1, 1])

        #plt.setp(plotPath, color=[(0, 0.44, 0.74), (0.85, 0.32, 0.09)])
        plt.show()

    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
        print ("mouse",button,state,x,y)
        if button==2:
            if state==0:
                print ([o.getName() for o in self.click_world(x,y)])
            return
        GLSimulationPlugin.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y,dx,dy):
        return GLSimulationPlugin.motionfunc(self,x,y,dx,dy)

if __name__ == "__main__":
    print( "gltemplate.py: This example demonstrates how to simulate a world and read user input")
    if len(sys.argv)<=1:
        print( "USAGE: simpleSim.py [world_file]")
        exit()

    viewer = MyGLViewer(sys.argv[1:])
    vis.run(viewer)
