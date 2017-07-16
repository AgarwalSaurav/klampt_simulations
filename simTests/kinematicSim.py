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

def getWall(dimX, dimY, dimZ, pos = [0, 0, 0], rotZ = 0):
        ## Get the wall geometry
	wall = Geometry3D()
	wall.loadFile("cube.off") # The wall is based upon cube primitive of unit dimension
        ## Not sure why the scaling is done through the rotation matrix, works though!
	wall.transform([dimX, 0, 0, 0, dimY, 0, 0, 0, dimZ], pos)
        rotMat = so3.rotation((0, 0, 1), math.radians(rotZ))
        wall.transform(rotMat, pos)
        return wall

def getWall_terrain(world, dimX, dimY, dimZ, pos = [0, 0, 0], nameWall="wall", color = [0.85, 0.85, 0.85, 1]):
        ## Attach a single wall to the world
        wall = getWall(dimX, dimY, dimZ, pos)
	world_wall = world.makeTerrain(nameWall)
	world_wall.geometry().set(wall)
        r = color[0]
        g = color[1]
        b = color[2]
        alpha = color[3]
	world_wall.appearance().setColor(r, g, b, alpha)
	return world_wall

def getDoubleRoomDoor(world, sc = 2, color = [0.85, 0.85, 0.85, 1]):
    ## Build a double room with a single door in the middle of the wall
    wall_thickness = 0.01
    w1 = getWall(4*sc, wall_thickness, 1, [-1*sc, -1*sc, 0], 0)
    w2 = getWall(wall_thickness, 4*sc, 1, [-1*sc, -1*sc, 0], 0)
    w3 = getWall(4*sc, wall_thickness, 1, [-1*sc, 1*sc, 0], 0)
    w4 = getWall(wall_thickness, 4*sc, 1, [1*sc, -1*sc, 0], 0)
    w5 = getWall(1.5*sc, wall_thickness, 1, [-1*sc, 0, 0], 0)
    w6 = getWall(1.5*sc, wall_thickness, 1, [0.25*sc, 0, 0], 0)
    DRDgeom = Geometry3D()
    DRDgeom.setGroup()
    for i,elem in enumerate([w1, w2, w3, w4, w5, w6]):
	g = Geometry3D(elem)
	DRDgeom.setElement(i,g)
    drd_setup = world.makeTerrain("DRD")
    drd_setup.geometry().set(DRDgeom)
    r = color[0]
    g = color[1]
    b = color[2]
    alpha = color[3]
    drd_setup.appearance().setColor(r, g, b, alpha)

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
    ## Add double room with a door. It should be before the vis.add line
    sc = 2 # Easier to scale according to terrain size
    getDoubleRoomDoor(world, sc)

    #add the world to the visualizer
    vis.add("world",world)

    #add the coordinate Manager to the visualizer
    #vis.add("coordinates",coordinates.manager())
    
    vp = vis.getViewport()
    vp.w,vp.h = 800,800
    vis.setViewport(vp)

    #do this if you want to test the robot configuration auto-fitting
    #vis.add("text1","Using a random configuration")
    setRandomSeed(int(time.time()))
    world.robot(0).randomizeConfig()
    

    ## Display the world coordinate system
    vis.add("WCS",[so3.identity(),[0,0,0]])
    vis.setAttribute("WCS", "size", 24)
    pt = [2,5,1]
    vis.add("some point",pt)
    #test an IKObjective
    link = world.robot(0).link(world.robot(0).numLinks()-1)
    #point constraint
    obj = ik.objective(link,local=[[0,0,0]],world=[pt])
    #hinge constraint
    obj = ik.objective(link,local=[[0,0,0],[0,0,0.1]],world=[pt,[pt[0],pt[1],pt[2]+0.1]])
    #transform constraint
    obj = ik.objective(link,R=link.getTransform()[0],t=pt)
    vis.add("ik objective",obj)

    #test the on-screen text display
    vis.addText("text2","Here's some red text")
    vis.setColor("text2",1,0,0)
    vis.addText("text3","Here's bigger text")
    vis.setAttribute("text3","size",24)
    vis.addText("text4","Transform status")
    vis.addText("textbottom","Text anchored to bottom of screen",(20,-30))

    #vis.addPlot('plot')
    #vis.setPlotDuration('plot',10.0)

    print "Visualization items:"
    vis.listItems(indent=2)

    vis.autoFitCamera()

    print "Starting visualization window..."
    #run the visualizer, which runs in a separate thread
    vis.setWindowTitle("Basic visualization test")

    vis.show()
    iteration = 0
    while vis.shown():
        vis.lock()
        #TODO: you may modify the world here.  This line tests a sin wave.
        #pt[2] = 1 + math.sin(iteration*0.03)
        vis.unlock()
        #changes to the visualization must be done outside the lock
        if (iteration % 100) == 0:
            if (iteration / 100)%2 == 0:
                #vis.hide("some blinking transform")
                vis.addText("text4","The transform was hidden")
                #vis.logPlotEvent('plot','hide')
            else:
                #vis.hide("some blinking transform",False)
                vis.addText("text4","The transform was shown")
                #vis.logPlotEvent('plot','show')
        #this is another way of changing the point's data
        #vis.add("some point",[2,5,1 + math.sin(iteration*0.03)],keepAppearance=True)
        time.sleep(0.01)
        iteration += 1
    vis.clearText()
    #vis.remove("plot")

    """
    #Now testing ability to re-launch windows
    print "Showing again..."
    vis.show()
    while vis.shown():
        time.sleep(0.01)
    """

    print "Doing a dialog..."
    vis.setWindowTitle("Dialog test")
    print "calling dialog()"
    vis.dialog()

    print "Doing a split screen program..."
    vp.w,vp.h = 640,480
    vis.setViewport(vp)
    for i in range(3):
        widgets = GLWidgetPlugin()
        widgets.addWidget(RobotPoser(world.robot(0)))
        #update the coordinates every time the widget changes
        widgets.widgetchangefunc = (lambda self:coordinates.updateFromWorld())
        vis.addPlugin(widgets)
    vis.setWindowTitle("Split screen test")
    vis.show()
    while vis.shown():
        time.sleep(0.1)
    
    print "Showing a dialog, back to normal..."
    vis.setPlugin(None)
    vis.dialog()
    print "Showing again, back to normal..."
    vis.setWindowTitle("Basic visualization test")
    vis.show()
    while vis.shown():
        time.sleep(0.01)

    print "Ending klampt.vis visualization."
    vis.kill()
