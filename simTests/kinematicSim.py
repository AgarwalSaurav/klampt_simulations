#!/usr/bin/python

import sys
from klampt import *
from klampt import vis
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.model import ik,coordinates
from klampt.math import so3
import time
import math
import buildWorld as bW

if __name__ == "__main__":
    print "vistemplate.py: This example demonstrates how to run the visualization framework"
    if len(sys.argv)<=1:
        print "USAGE: vistemplate.py [world_file]"
        exit()

    #creates a world and loads all the items on the command line
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)

    coordinates.setWorldModel(world)

    ## Get walls
    #bW.getDoubleRoomWindow(world, 8, 8, 1.2)
    bW.getDoubleRoomDoor(world, 8, 8, 1)
    #add the world to the visualizer
    vis.add("world",world)

    #add the coordinate Manager to the visualizer
    #vis.add("coordinates",coordinates.manager())
    
    vp = vis.getViewport()
    vp.w,vp.h = 800,800
    vis.setViewport(vp)

    #do this if you want to test the robot configuration auto-fitting
    #vis.add("text1","Using a random configuration")
    #setRandomSeed(int(time.time()))
    #world.robot(0).randomizeConfig()
    

    ## Display the world coordinate system
    vis.add("WCS",[so3.identity(),[0,0,0]])
    vis.setAttribute("WCS", "size", 24)

    link = world.robot(0).link(0)
    ## Setup a 6 DoF point robot
    pt = [0, 0, 0]
    rotVec = [0, 0, 1]
    rotAng = 0
    rotMat = so3.rotation(rotVec, rotAng)
    vis.add("pt",[rotMat, pt])
    vis.setAttribute("pt", "size", 32)
    vis.edit("pt")
    #test the on-screen text display
    vis.addText("text2","Here's some red text")
    vis.setColor("text2",1,0,0)
    vis.addText("text3","Here's bigger text")
    vis.setAttribute("text3","size",24)
    vis.addText("text4","Transform status")
    vis.addText("textbottom","WCS: X-axis Red, Y-axis Green, Z-axis Blue",(20,-30))

    #vis.addPlot('plot')
    #vis.setPlotDuration('plot',10.0)

    print "Visualization items:"
    vis.listItems(indent=2)

    vis.autoFitCamera()

    print "Starting visualization window..."
    #run the visualizer, which runs in a separate thread
    vis.setWindowTitle("Visualization for kinematic simulation")

    vis.show()
    iteration = 0
    while vis.shown():
        vis.lock()
        #TODO: you may modify the world here.
        pt[0] = math.sin(time.time())
        pt[1] = math.cos(time.time())
        pt[2] = 0.5
        tht = time.time()
        phi = math.pi/4.0 + time.time()
        alpha = math.pi/3.0 + time.time()
        rotVec = [math.sin(tht)*math.cos(phi), math.sin(tht)*math.sin(phi), math.cos(tht)]
        rotAng = math.pi*math.sin(alpha)
        rotMat = so3.rotation(rotVec, rotAng)
        #vis.remove("pt")
        vis.add("pt",[rotMat, pt], keepAppearance=True)
        vis.unlock()
        vis.edit("pt")
        #changes to the visualization must be done outside the lock
        time.sleep(0.01)
    vis.clearText()

    print "Ending klampt.vis visualization."
    vis.kill()
