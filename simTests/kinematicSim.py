#!/usr/bin/python
## The file demonstrates:
##   1. Adding rooms and walls to the environment (refer to buildWorld.py as well)
##   2. Setting up a robot
##   3. Perform collision checking
##   4. Modify the robot configurations and visualize
##   5. Adding text objects and modifying them

import sys
from klampt import *
from klampt import vis
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.model import ik,coordinates
from klampt.math import so3
import klampt.model.collide as collide
import time
import math
import buildWorld as bW
sys.path.append("./kinematics/")
from sphero6DoF import sphero6DoF
from kobukiHolonomic import kobukiHolonomic
from decimal import Decimal

if __name__ == "__main__":
    if len(sys.argv)<=1:
        print "USAGE: kinematicSim.py [world_file]"
        exit()

    ## Creates a world and loads all the items on the command line
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)

    coordinates.setWorldModel(world)

    ## Get walls
    #bW.getDoubleRoomWindow(world, 8, 8, 1.2)
    bW.getDoubleRoomDoor(world, 8, 8, 1)

    ## Add the world to the visualizer
    vis.add("world",world)

    vp = vis.getViewport()
    vp.w,vp.h = 1200,800
    vis.setViewport(vp)

    ## Create robot object. Change the class to the desired robot. 
    ## Also, make sure the robot class corresponds to the robot in simpleWorld.xml file
    robot = kobukiHolonomic(world.robot(0), vis)
    robot.setAltitude(0.1)
    #robot = turtlebot(world.robot(0), vis)
    #robot = sphero6DoF(world.robot(0), vis)

    ## Display the world coordinate system
    vis.add("WCS",[so3.identity(),[0,0,0]])
    vis.setAttribute("WCS", "size", 24)


    #vis.addPlot('plot')
    #vis.setPlotDuration('plot',10.0)

    print "Visualization items:"
    vis.listItems(indent=2)

    vis.autoFitCamera()
    print("Collision check:")
    vis.addText("textCol", "No collision")
    vis.setAttribute("textCol","size",24)
    collisionFlag = False
    collisionChecker = collide.WorldCollider(world)
    for i,j in collisionChecker.collisionTests():
        if i[1].collides(j[1]):
            collisionFlag = True
            strng = "Object "+i[0].getName()+" collides with "+j[0].getName()
            print(strng)
            vis.addText("textCol", strng)

    ## On-screen text display
    vis.addText("textConfig","Robot configuration: ")
    vis.setAttribute("textConfig","size",24)
    vis.addText("textbottom","WCS: X-axis Red, Y-axis Green, Z-axis Blue",(20,-30))

    print "Starting visualization window#..."

    ## Run the visualizer, which runs in a separate thread
    vis.setWindowTitle("Visualization for kinematic simulation")

    collisionFlag = collisionChecker.robotTerrainCollisions(world.robot(0), world.terrain(0))
    print(collisionFlag)
    #print(next(collisionFlag))
    vis.show()
    simTime = 30
    startTime = time.time()
    while vis.shown() and (time.time() - startTime < simTime):
        vis.lock()
        ## You may modify the world here.
        ## Specifying change in configuration of the robot

        ## 6DoF spherical robot
        #q = robot.getConfig()
        #q[0] = math.sin(time.time())
        #q[1] = math.cos(time.time())
        #q[2] = 0.5
        #q[3] = 2 * math.pi * (math.cos(time.time()) + 1)
        #q[4] = 2 * math.pi * (math.sin(time.time()) + 1)
        #q[5] = 2 * math.pi * (math.sin(time.time() + math.pi/4.0) + 1)

        ## 3DoF holonomic kobuki
        q = robot.getConfig()
        q[0] = math.cos(time.time())
        q[1] = math.sin(time.time())
        q[2] = 2 * math.pi * (math.cos(time.time()) + 1)
        
        ## Turtlebot 
        #q = robot.getConfig()
        #q[0] = math.cos(time.time())
        #q[1] = math.sin(time.time())
        #q[2] = 2 * math.pi * (math.cos(time.time()) + 1)

        robot.setConfig(q)
        q2f = [ '{0:.2f}'.format(elem) for elem in q]
        strng = "Robot configuration: " + str(q2f)
        vis.addText("textConfig", strng)

        ## Checking collision
        collisionFlag = False
        for i,j in collisionChecker.collisionTests():
            if i[1].collides(j[1]):
                collisionFlag = True
                strng = "Object "+i[0].getName()+" collides with "+j[0].getName()
                print(strng)
                vis.addText("textCol", strng)
                vis.setColor("textCol", 0.8500, 0.3250, 0.0980)

        if not collisionFlag:
            vis.addText("textCol", "No collision")
            vis.setColor("textCol", 0.4660, 0.6740, 0.1880)

        vis.unlock()
        #changes to the visualization must be done outside the lock
        time.sleep(0.01)
    vis.clearText()

    print "Ending klampt.vis visualization."
    vis.kill()
