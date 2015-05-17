from matplotlib import pyplot as plt
from numpy import sin, cos, pi
import random


theta = [(pi/18, 7./18 * pi), (pi/18, 7./18 * pi)]
l1, l2 = 1, 01


def calc_point(angle1, angle2):
    x = cos(angle1) * l1
    y = sin(angle1) * l1

    # Angle to horizontal
    alpha = angle1 - angle2
    x += cos(alpha) * l2
    y += sin(alpha) * l2

    return (x,y)


points = [(0,0)]
for i in range(50000):
    angle1 = random.uniform(theta[0][0],theta[0][1])
    angle2 = random.uniform(theta[1][0],theta[1][1])
    #print 180/pi*angle1, 180/pi*angle2
    points.append(calc_point(angle1, angle2))


plt.scatter(*zip(*points),marker="+")



theta = [(pi/18, pi/18), (-pi, pi)]
points = [(0,0)]
for i in range(50000):
    angle1 = random.uniform(theta[0][0],theta[0][1])
    angle2 = random.uniform(theta[1][0],theta[1][1])
    #print 180/pi*angle1, 180/pi*angle2
    points.append(calc_point(angle1, angle2))


plt.scatter(*zip(*points),marker=",")

theta = [(7*pi/18, 7*pi/18), (-pi, pi)]
points = [(0,0)]
for i in range(50000):
    angle1 = random.uniform(theta[0][0],theta[0][1])
    angle2 = random.uniform(theta[1][0],theta[1][1])
    #print 180/pi*angle1, 180/pi*angle2
    points.append(calc_point(angle1, angle2))


plt.scatter(*zip(*points),marker=",")


theta = [(-pi, pi), (pi/18, pi/18)]
points = [(0,0)]
for i in range(50000):
    angle1 = random.uniform(theta[0][0],theta[0][1])
    angle2 = random.uniform(theta[1][0],theta[1][1])
    #print 180/pi*angle1, 180/pi*angle2
    points.append(calc_point(angle1, angle2))


plt.scatter(*zip(*points),marker=",")


theta = [(-pi, pi), (7*pi/18, 7*pi/18)]
points = [(0,0)]
for i in range(50000):
    angle1 = random.uniform(theta[0][0],theta[0][1])
    angle2 = random.uniform(theta[1][0],theta[1][1])
    #print 180/pi*angle1, 180/pi*angle2
    points.append(calc_point(angle1, angle2))


plt.scatter(*zip(*points),marker=",")



plt.axis('equal')
plt.show()
