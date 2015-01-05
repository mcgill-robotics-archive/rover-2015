#!/usr/bin/python
#This program will simulate the rover using pyglet

#fixes scaling
from __future__ import division
from pyglet import clock, font, image, window
from pyglet.gl import *
#reads in values from joystick
import rospy
from control_systems.msg import SetPoints
import math

D = 50e-2  # distance between wheels of: front and middle/middle and rear[m]   
B = 40e-2  # distance between longitudinal axis and port/startboard wheels[m]
R = 16.5e-2 # wheel radius [m]
W = 15e-2 # wheel width [m]

#All the stuff flat on the screen (text, statistics)
class Hud(object):

    def __init__(self, win):
        helv = font.load('Helvetica', win.width / 50.0)
        #Create title for name of simulator
        self.text = font.Text(
            helv,
            'McGill Robotics Rover Simulator',
            x=win.width / 2,
            y=win.height -win.height//25,
            halign=font.Text.CENTER,
            valign=font.Text.CENTER,
            color=(1, 1, 1, 0.5),
        )
        #Display fps
        self.fps = clock.ClockDisplay()
        glClearColor(1.,1.,1.,1.);

    def draw(self):
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
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

    def __init__(self, id, width, height, x, y, rot, rotO, col):
        #default colour is black
        self.id = id
        self.width = width
        self.height = height
        self.x = x
        self.y = y
        self.rot = rot
        self.rotO = rotO
        self.col = col

    def draw(self):
        glLoadIdentity()
        glRotatef(self.rotO, 0, 0, 1)
        glTranslatef(self.x, self.y, 0.0)
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
        #spawns 10 triangles
        self.spawnEntity(75*2*B,75*2*D,0,0,0,0,(0.5,0.5,0.5,1.0))
        #FL
        self.spawnEntity(75*W,75*2*R,75*-B,75*D,0,0)
        #FR
        self.spawnEntity(75*W,75*2*R,75*B,75*D,0,0)
        #RL
        self.spawnEntity(75*W,75*2*R,-75*B,-75*D,0,0)
        #RR
        self.spawnEntity(75*W,75*2*R,75*B,-75*D,0,0)
        #clock.schedule_interval(self.spawnEntity, 0.25)

    def spawnEntity(self, width, height, x, y, rot, rotO,
        col = (0., 0., 0., 1.)):
        ent = Entity(self.nextEntId, width, height, x, y, rot, rotO, col)
        self.ents[ent.id] = ent
        self.nextEntId += 1
        return ent

    def ORotate(self, id, theta):
        self.ents.values()[id].rotO = -180*theta/math.pi

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
        rospy.init_node('simulator')
        rospy.Subscriber('/wheels',SetPoints, self.update_wheels)
        self.FL = 0
        self.FR = 0
        self.RL = 0
        self.RR = 0
        #start opengl
        self.world = World()
        self.win = window.Window(fullscreen=False, vsync=True)
        self.camera = Camera(self.win, zoom=100.0)
        self.hud = Hud(self.win)

    def update_wheels(self,msg):
        #load in values
        self.FL = msg.thetaFL
        self.FR = msg.thetaFR
        self.RL = msg.thetaRL
        self.RR = msg.thetaRR

    def run(self):
        r=rospy.Rate(60)
        while not rospy.is_shutdown() and not self.win.has_exit:
            self.win.dispatch_events()
            #rotate entire body
            for x in range(5):
                self.world.ORotate(x,0.)
            #turn wheels according to values
            self.world.pointRotate(1,self.FL)
            self.world.pointRotate(2,self.FR)
            self.world.pointRotate(3,self.RL)
            self.world.pointRotate(4,self.RR)
            #Draw contents
            self.camera.worldProjection()
            self.world.draw()
            self.camera.hudProjection()
            self.hud.draw()
            #Move one step forward
            self.win.flip()
            r.sleep()


app = App()
app.run()
rospy.spin()

