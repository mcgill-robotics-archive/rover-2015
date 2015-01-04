#!/bin/sh
#This program will simulate the rover using pyglet

#fixes scaling
from __future__ import division
from pyglet import clock, font, image, window
from pyglet.gl import *

D = 50e-2  # distance between wheels of: front and middle/middle and rear[m]   
B = 40e-2  # distance between longitudinal axis and port/startboard wheels[m]
R = 16.5e-2 # wheel radius [m]
W = 15e-2 # wheel width [m]

class Hud(object):

    def __init__(self, win):
        helv = font.load('Helvetica', win.width / 50.0)
        self.text = font.Text(
            helv,
            'McGill Robotics Rover Simulation',
            x=win.width / 2,
            y=win.height -win.height//25,
            halign=font.Text.CENTER,
            valign=font.Text.CENTER,
            color=(1, 1, 1, 0.5),
        )
        self.fps = clock.ClockDisplay()
        glClearColor(1.,1.,1.,1.);

    def draw(self):
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        self.text.draw()
        self.fps.draw()

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
        self.ents.values()[id].rotO += theta

    def pointRotate(self, id, theta):
        self.ents.values()[id].rot += theta

    def draw(self):
        glClear(GL_COLOR_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        for ent in self.ents.values():
            ent.draw()

class App(object):

    def __init__(self):
        self.world = World()
        self.win = window.Window(fullscreen=True, vsync=True)
        self.camera = Camera(self.win, zoom=100.0)
        self.hud = Hud(self.win)
        clock.set_fps_limit(60)

    def mainLoop(self):
        while not self.win.has_exit:
            self.win.dispatch_events()
            for x in range(5):
                self.world.ORotate(x,0.25)
            for x in range(1,5):
                self.world.pointRotate(x,1)
            self.camera.worldProjection()
            self.world.draw()

            self.camera.hudProjection()
            self.hud.draw()

            clock.tick()
            self.win.flip()

app = App()
app.mainLoop()

#batch = g.graphics.Batch()
#g.graphics.draw(2,g.gl.GL_POINTS,
#            ('v2i', (10, 15, 30, 35)))

