#!/usr/bin/python
#This program will simulate the rover using pyglet

#fixes scaling
from __future__ import division
from pyglet import clock, font, image, window
from pyglet.gl import *
#reads in values from joystick
import rospy
from control_systems.msg import ArmAngles
import math
import sys

#import lengths of robotic arm
#length of first part of arm
a1 = rospy.get_param('control/ln_upperarm',0.5)
#length of second part of arm
a2 = rospy.get_param('control/ln_forearm',0.5)

zero =1e-10

def sgn (x):
    if abs(x) < zero:
        return 0.
    else:
        return float(x)/abs(float(x))

#All the stuff flat on the screen (text, statistics)
class Hud(object):

    def __init__(self, win):
        helv = font.load('Helvetica', win.width / 50.0)
        helvSmall = font.load('Helvetica', win.width / 80.0)
        #Create title for name of simulator
        self.text = font.Text(
            helv,
            'McGill Robotics Arm Simulator',
            x=win.width / 2,
            y=win.height -win.height//25,
            halign=font.Text.CENTER,
            valign=font.Text.CENTER,
            color=(0., 0., 0., 1.),
        )
        self.angles = [
        font.Text(
            helvSmall,
            y,
            x=10,
            y=win.height - x*11-20,
            halign=font.Text.LEFT,
            valign=font.Text.TOP,
            color=z) 
        for x,y,z in zip(
            range(1,5),
            ['Wheels:',
            'Shoulder Elevation: '+str(0.0)+' rad',
            'Shoulder Orientation: '+str(0.0)+' rad',
            'Elbow Elevation: '+str(0.0)+' rad'
            ],
            [(0.,0.,0.,1.),
            (0.5,0.,0.,1.),
            (0.,0.5,0.,1.),
            (0.,0.,0.5,1.)
            ]
            )
        ]


        #Display fps
        self.fps = clock.ClockDisplay()
        glClearColor(0.929,0.788,0.686,1.);

    def update(self,set,win):
        helvSmall = font.load('Helvetica', win.width / 80.0)

        self.angles = [
        font.Text(
            helvSmall,
            y,
            x=10,
            y=win.height - x*11-20,
            halign=font.Text.LEFT,
            valign=font.Text.TOP,
            color=z) 
        for x,y,z in zip(
            range(1,5),
            ['Wheels:',
            'Shoulder Elevation: '+str(set[0])+' rad',
            'Shoulder Orientation: '+str(set[1])+' rad',
            'Elbow Elevation: '+str(set[2])+' rad'
            ],
            [(0.,0.,0.,1.),
            (0.5,0.,0.,1.),
            (0.,0.5,0.,1.),
            (0.,0.,0.5,1.)
            ]
            )
        ]
    

    def draw(self):
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        for x in self.angles:
            x.draw()
        self.text.draw()
        self.fps.draw()

#The graphics objects
class Camera(object):

    def __init__(self, win, zoom=1.0):
        self.win = win
        self.zoom = zoom

    def worldProjection(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        widthRatio = self.win.width / self.win.height
        gluOrtho2D(
            -self.zoom * widthRatio,
            self.zoom * widthRatio,
            -self.zoom,
            self.zoom)

    def hudProjection(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluOrtho2D(0, self.win.width, 0, self.win.height)

class Entity(object):

    def __init__(self, id, width, height, x, y, rot, rotP,rotPoint,
        rotP2, rotPoint2, col, dir, trans):
        #default colour is black
        self.id = id
        self.width = width
        self.height = height
        self.x = x
        self.y = y
        self.rot = rot
        self.rotP = rotP
        self.rotPoint = rotPoint
        self.rotP2 = rotP2
        self.rotPoint2 = rotPoint
        self.col = col
        self.dir = dir
        self.trans = trans

    def draw(self):
        glLoadIdentity()
        glTranslatef(self.x, self.y, 0.)
        #rotate around specified point
        glRotatef(self.rotP2,0,0,1)
        glTranslatef(self.rotPoint2[0],self.rotPoint2[1],0.)
        #glRotatef(self.rotP2, 0, 0, 1)
        #glTranslatef(self.rotPoint2[0]/2-self.rotPoint[0]/2,
        #    self.rotPoint2[1]/2-self.rotPoint[1]/2,0.)
        glRotatef(self.rotP, 0, 0, 1)
        glTranslatef(self.rotPoint[0]/2,self.rotPoint[1]/2,0.)

        glTranslatef(self.trans[0],self.trans[1],0.)
        #rotate things around their centres (wheels)
        glRotatef(self.rot, 0, 0, 1)
        glScalef(self.width, self.height, 1.0)
        glBegin(GL_QUADS)
        glColor4f(*self.col)
        glVertex2f(-0.5, 0.5)
        glColor4f(*self.col)
        glVertex2f(-0.5, -0.5)
        glColor4f(*self.col)
        glVertex2f(0.5, -0.5)
        glColor4f(*self.col)
        glVertex2f(0.5, 0.5)
        glEnd()

class World(object):

    def __init__(self):
        self.ents = {}
        self.nextEntId = 0
        #spawns parts of rover
        #body
        self.spawnEntity(4*a1,150*a1,-100*a1,-100*a1,0,0,(0,0),
            0,(0.,0.),(0.,0.,0.,1.),0., (0,0))
        self.spawnEntity(4*a1,150*a1,-100*a1,-100*a1,4,0,(0,0),
            0,(0.,0.),(0.,0.,0.,1.),0., (0,0))


    def spawnEntity(self, width, height, x, y, rot, rotP,rotPoint,
            rotP2, rotPoint2, col, dir, trans):
        ent = Entity(self.nextEntId, width, height, x, y, rot, rotP,rotPoint,
                rotP2, rotPoint2, col, dir, trans)
        self.ents[ent.id] = ent
        self.nextEntId += 1
        return ent

    def translate(self, id, translation):
        self.ents.values()[id].trans = translation

    def PRotate(self, id, theta, point):
        self.ents.values()[id].rotP = -180*theta/math.pi
        self.ents.values()[id].rotPoint = point

    def PRotate2(self, id, theta, point):
        self.ents.values()[id].rotP2 = -180*theta/math.pi
        self.ents.values()[id].rotPoint2 = point

    def pointRotate(self, id, theta):
        self.ents.values()[id].rot = -180*theta/math.pi

    def draw(self):
        glClear(GL_COLOR_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        for ent in self.ents.values():
            ent.draw()

class App(object):

    def __init__(self):
        #inititate ros part
        rospy.init_node('arm_simulator')
        rospy.Subscriber('/arm',ArmAngles, self.update_arm)
        #angles of rover robtic arm
        self.arm = ArmAngles()
        # set all angles to zero
        self.arm.shoulderOrientation = 0
        self.arm.shoulderElevation = 0
        self.arm.elbow = 0
        self.arm.wristOrientation = 0
        self.arm.wristElevation = 0

        #body rotation of entire rover
        self.rotation = 0
        #start opengl
        self.world = World()
        self.win = window.Window(fullscreen=False, vsync=True)
        self.win.width
        self.camera = Camera(self.win, zoom=100.0)
        self.hud = Hud(self.win)

    def update_arm(self, msg):
        #load in values from arm
        self.arm.shoulderOrientation = msg.shoulderOrientation
        self.arm.shoulderElevation = msg.shoulderElevation
        self.arm.elbow = msg.elbow
        self.arm.wristOrientation = msg.wristOrientation
        self.arm.wristElevation = msg.wristElevation

    def run(self):
        r=rospy.Rate(60)
        while not rospy.is_shutdown() and not self.win.has_exit:

            self.win.dispatch_events()


            self.world.PRotate(0,self.arm.shoulderElevation,(0,150*a1))
            self.world.PRotate(1,self.arm.shoulderElevation,(0,150*a1))
            self.world.pointRotate(1,-self.arm.elbow)
            psi = 5*math.pi/2+self.arm.elbow
            self.world.translate(1,(150*a1*math.cos(psi)/2,(1/2+math.sin(psi)/2)*150*a1))
            #self.world.translate(1,(70*a1,55*a1))

            #Draw contents
            self.camera.worldProjection()
            self.world.draw()
            self.camera.hudProjection()
            self.hud.update(
                [self.arm.shoulderElevation,
                self.arm.shoulderOrientation,
                -self.arm.elbow],
                self.win)
            self.hud.draw()
            #Move one step forward
            self.win.flip()
            r.sleep()
        sys.exit()



app = App()
app.run()
rospy.spin()

