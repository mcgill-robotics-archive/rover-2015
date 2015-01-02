#!/bin/sh
#This program will draw a very simple geometric outline and
#wheels and turn them and the rover according to the inputted
#data, to ease understanding of the wheel data

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib as mpl
import matplotlib.patches as patches
import math

plt.ion()

theta = math.pi/2 + math.pi/4
s = math.sin(theta)
c = math.cos(theta)
verts = [
    ( s-c, -s-c), # left, bottom
    (-s-c, -s+c), # left, top
    ( c-s,  c+s), # right, top
    ( c+s,  s-c), # right, bottom
    (0, 0), # ignored
    ]

codes = [Path.MOVETO,
         Path.LINETO,
         Path.LINETO,
         Path.LINETO,
         Path.CLOSEPOLY,
         ]

path = Path(verts, codes)

fig = plt.figure()
ax = fig.add_subplot(111)
patch = patches.PathPatch(path, facecolor='orange', lw=2)
ax.add_patch(patch)
ax.set_xlim(-2,2)
ax.set_ylim(-2,2)

for x in range(100):
    s = math.sin(x/10.)
    c = math.cos(x/10.)
    verts = [
        ( s-c, -s-c), # left, bottom
        (-s-c, -s+c), # left, top
        ( c-s,  c+s), # right, top
        ( c+s,  s-c), # right, bottom
        (0, 0), # ignored
        ]
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor='orange', lw=2)
    fig.delaxes(ax)
    ax = fig.add_subplot(111)
    ax.add_patch(patch)
    ax.set_xlim(-2,2)
    ax.set_ylim(-2,2)
    plt.clf
    plt.draw()
    plt.show()

