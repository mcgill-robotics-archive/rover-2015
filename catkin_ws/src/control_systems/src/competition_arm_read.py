#!/usr/bin/env python

import rospy
import pygame
from pygame.locals import *
from math import sqrt, atan, pi, cos, sin
import rospy
from control_systems.msg import ArmMotion, ArmAngles
from numpy import random

# a1 is the length of the upper arm (touches base)
a1 = rospy.get_param('control/ln_upperarm', 0.439)
# a2 is the length of the forearm (attached to hand)
a2 = rospy.get_param('control/ln_forearm', 0.149)
# a3 is the length of the wrist (attached to glove)
a3 = rospy.get_param('control/ln_wrist', 0.189)

#To help avoid divisions by close to zero
zero = 1e-10

# bounds on forearm and upperarm angles
forMin = pi/18  # rospy.get_param('control/bound_lower_forearm',-30*pi/36)
forMax = 7*pi/18  # rospy.get_param('control/bound_upper_forearm',31*pi/36)
uppMin = pi/18  # rospy.get_param('control/bound_lower_upperarm',pi/18)
uppMax = 7*pi/18  # rospy.get_param('control/bound_upper_upperarm',8*pi/18)
rotMin = -pi  # rospy.get_param('control/bound_lower_orientation',-7*pi/8)
rotMax = pi # rospy.get_param('control/bound_upper_orientation',7*pi/8)
#wrist!
wriMin = -pi/2 
wriMax = pi/2

masterPoints = []

class ArmControlReader(object):
    def __init__(self):
        # initiate node
        rospy.init_node('arm_reader')
        #Start pygame module
        self.winMaxX = 1000
        self.winMaxY = 1000
        self.winX = 0
        self.winY = 0
        self.winMessage = ArmMotion()
        self.winMessage.x = 0
        self.winMessage.y = 0
        self.winMessage.theta = 0
        self.winMessage.phi = 0
        self.winMessage.on = False
        self.winMessage.cartesian = False
        pygame.init()
        #initialize the screen
        self.screen = pygame.display.set_mode((self.winMaxX/2,self.winMaxY))
        pygame.display.set_caption('Arm Control')
        #fill the background
        self.background = pygame.Surface(self.screen.get_size())
        self.background = self.background.convert()
        self.background.fill((250,250,250))
        # Blit everything to the screen
        self.screen.blit(self.background,(0, 0))
        pygame.display.flip()

        # publish arm settings to this topic
        self.pubArm = rospy.Publisher('/arm', ArmAngles, queue_size=10, 
                                      latch=10)
        # settings to be read in
        self.settings = ArmMotion()
        # set values to off
        self.settings.x = 0
        self.settings.y = 0
        self.settings.theta = 0
        self.settings.phi = 0
        self.settings.on = False
        self.settings.cartesian = False
        #velocity of the arm
        self.settings.velocity = False
        # angles:
        # angle at base
        self.angles = ArmAngles()
        # set all angles to zero
        self.angles.shoulderOrientation = 0
        self.angles.shoulderElevation = 0
        self.angles.elbow = 0
        self.angles.wristOrientation = 0
        #wrist angle!!
        self.angles.wristElevation = 0
        rospy.Subscriber('/cmd_arm', ArmMotion, self.update_settings,
                         queue_size=10)

        #Define corners of curved arm boundary (help optimise boundary process)
        self.topCorner   = (a1*cos(uppMax)+a2*cos(uppMax-forMin),
                            a1*sin(uppMax)+a2*sin(uppMax-forMin))
        self.rightCorner = (a1*cos(uppMin)+a2*cos(uppMin-forMin),
                            a1*sin(uppMin)+a2*sin(uppMin-forMin))
        self.bottomCorner= (a1*cos(uppMin)+a2*cos(uppMin-forMax),
                            a1*sin(uppMin)+a2*sin(uppMin-forMax))
        self.leftCorner  = (a1*cos(uppMax)+a2*cos(uppMax-forMax),
                            a1*sin(uppMax)+a2*sin(uppMax-forMax))

        #safe value to start
        self.settings.x = self.topCorner[0]
        self.settings.y = self.topCorner[1]
        self.winMessage.x = self.topCorner[0]
        self.winMessage.y = self.topCorner[1]

        #Circles used to speed up boundary detection
        #Defined as (centre)=(a,b), radius=r, miny, maxy
        #o-outer,i-inner, t-top,b-bottom circle

        #for our situation, the ot circle
        #completely encloses all region, and
        #the it and ib circles do not contain
        #any of the moveable distance.
        #half of ob excludes any other region
        #from ot.
        self.ot = [(0,0), distance(*self.topCorner)]
        self.ob = [(a1*cos(uppMin),a1*sin(uppMin)), a2]
        self.it = [(a1*cos(uppMax),a1*sin(uppMax)), a2]
        self.ib = [(0,0), distance(*self.bottomCorner)]

        
        #update to safe
        self.update_settings(self.winMessage)

    def update_window_control(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                return
        self.screen.blit(self.background, (0, 0))
        pygame.display.flip()
        #update position of box
        if pygame.mouse.get_pressed()[0]:
            (self.winX,self.winY) = pygame.mouse.get_pos()
            (x,y) = (2.*(float(self.winX))/self.winMaxX,
                1.-2*self.winY/float(self.winMaxY))
            self.winMessage.x = x
            self.winMessage.y = y
            #try to update settings
            self.update_settings(self.winMessage)

    def update_settings(self, msg):
        # import readings into object
        # self.settings.x = msg.x
        # self.settings.y = msg.y
        self.settings.cartesian = msg.cartesian
        self.settings.velocity = msg.velocity
        if self.settings.cartesian:
            # ################################################################
            # needs protection against div/0
            msg.x, msg.y, msg.theta = convertCartesian(msg.x, msg.y, msg.theta)
        # If user is uploading velocity coordinates (velocity)
        if self.settings.velocity:
            # add on old settings
            msg.x += self.settings.x
            msg.y += self.settings.y
            msg.theta += self.settings.theta

        #go to closest orientation value
        if rotMin<=msg.theta<=rotMax:
            self.settings.theta = msg.theta
        #else, go to closest
        elif abs(msg.theta-rotMax)<=abs(msg.theta-rotMin):
            self.settings.theta = rotMax
        else:
            self.settings.theta = rotMin


        #Whether entered point is out of bounds or not (assumed it is until 
        #proven innocent)
        outOfBounds = True
        #make sure point can be reached
        if a1+a2 >= distance(msg.x,msg.y) >= abs(a1-a2) and msg.x>0:
           #Following function should not throw an error in this case.
            getAngles = possibleAngles(msg.x,msg.y)
            if self.anglesOkay(getAngles[1][0], getAngles[1][1]):
                #Select final angle set
                finalAngles = getAngles[1]
                self.angles.elbow = finalAngles[1]
                self.angles.shoulderElevation = finalAngles[0]
                outOfBounds = False
            elif self.anglesOkay(getAngles[0][0],getAngles[0][1]):
                finalAngles=getAngles[0]
                self.angles.elbow = finalAngles[1]
                self.angles.shoulderElevation = finalAngles[0]
                outOfBounds = False

        if outOfBounds:
            #quick bound check
            #Need to find closest point...
            #If not within bounds, we will
            #find the closest point within bounds!
            #This has been optimized using Mathematica and monte carlo
            points = self.circlePoints((msg.x,msg.y))
            #Corners may also be extremum
            points.append(self.topCorner)
            points.append(self.rightCorner)
            points.append(self.bottomCorner)
            points.append(self.leftCorner)
            #print points[:2]
            #Find the nearest valid point
            s = [ddistance(points[0],(msg.x,msg.y)),points[0]]
            for i in points[1:]:
                tmp = ddistance(i,(msg.x,msg.y))
                if tmp < s[0]:
                    s = [tmp,i]
            #This is the closest valid point to the requested
            #masterPoints.append(points)
            self.settings.x = s[1][0]          
            self.settings.y = s[1][1]
            getAngles = possibleAngles(self.settings.x,self.settings.y)
            if self.anglesOkay(getAngles[1][0], getAngles[1][1]):
                #Select final angle set
                finalAngles = getAngles[1]
                self.angles.elbow = finalAngles[1]
                self.angles.shoulderElevation = finalAngles[0]
                outOfBounds = False
            elif self.anglesOkay(getAngles[0][0],getAngles[0][1]):
                finalAngles=getAngles[0]
                self.angles.elbow = finalAngles[1]
                self.angles.shoulderElevation = finalAngles[0]
                outOfBounds = False
        else:
            self.settings.x = msg.x
            self.settings.y = msg.y


        self.angles.shoulderOrientation = self.settings.theta

        #Calculate wrist angle after testing
        testAngle = msg.phi - self.angles.shoulderElevation + self.angles.elbow
        if wriMax>=testAngle>=wriMin:
            self.settings.phi = msg.phi
            self.angles.wristElevation = testAngle

        #function will publish at 60Hz

    def anglesOkay(self, uppAng, forAng):
        if forMin<=forAng<=forMax and uppMin<=uppAng<=uppMax:
            return True
        return False

    def circlePoints(self,(x,y)):
        #function gives the closest viable points on circles
        points = []
        #get closest point on circle to user's point
        testPoint = self.closePoint(self.ot[0],self.ot[1],(x,y))
        #check y-coords if within region
        if self.topCorner[1]>=testPoint[1]>=self.rightCorner[1] \
            and max(self.topCorner[0],self.rightCorner[0])>=testPoint[0]\
            and min(self.topCorner[0],self.rightCorner[0])<=testPoint[0]:
            #append to viable points
            points.append(testPoint)
        #repeat
        testPoint = self.closePoint(self.ob[0],self.ob[1],(x,y))
        if self.rightCorner[1]>=testPoint[1]>=self.bottomCorner[1] \
            and max(self.rightCorner[0],self.bottomCorner[0])>=testPoint[0]\
            and min(self.rightCorner[0],self.bottomCorner[0])<=testPoint[0]:

            points.append(testPoint)
        testPoint = self.closePoint(self.it[0],self.it[1],(x,y))
        if self.topCorner[1]>=testPoint[1]>=self.leftCorner[1] \
            and max(self.topCorner[0],self.leftCorner[0])>=testPoint[0]\
            and min(self.topCorner[0],self.leftCorner[0])<=testPoint[0]:

            points.append(testPoint)
        testPoint = self.closePoint(self.ib[0],self.ib[1],(x,y))
        if self.leftCorner[1]>=testPoint[1]>=self.bottomCorner[1] \
            and max(self.leftCorner[0],self.bottomCorner[0])>=testPoint[0]\
            and min(self.leftCorner[0],self.bottomCorner[0])<=testPoint[0]:

            points.append(testPoint)

        return points

    def closePoint(self, (a,b),r,(x,y)):
        #of equation (x-a)^2+(y-b)^2=r^2 to the point (x,y)
        #y2 = b+r*(y-b)/ddistance((a,b),(x,y))
        #x2= a+sqrt(r**2-(y2-b)**2)
        #return (x2,y2)

        #new test code
        v = (x-a,y-b)
        #normalize
        av = distance(v[0],v[1])
        if abs(av) < zero:
            #return nonsensical (working) value
            #corner values will catch this
            return (a,b)
        return (a+r*v[0]/av,b+r*v[1]/av)

    #Checks if value is inside curved region (see images)
    def withinBounds(self,(x,y)):
        #initial dummy checks:
        #check if outside rectangle boundary
        if not (max(self.topCorner[1], self.leftCorner[1],
            self.rightCorner[1])>=y and\
            min(self.bottomCorner[1], self.leftCorner[1],
                self.rightCorner[1]) <= y):
            return False
        if not (min(self.topCorner[0],
                    self.rightCorner[0],self.bottomCorner[0])<=x and\
                max(self.topCorner[0],self.leftCorner[0],
                    self.bottomCorner[0])>=x):
            return False
        #circle checks - all of these observed using the Monte Carlo Method
        #check if outside largest circle (validity is inside)
        #or inside circle ib (validity is outside)
        if not (self.ot[1]>=distance(x,y)>=self.ib[1]):
            return False
        #check if inside circle it (validity is outside)
        if not (ddistance(self.it[0],(x,y))>=self.it[1]):
            return False
        #If below right corner, must be within circle ob.
        if (y < self.rightCorner[1] and\
            ((ddistance(self.ob[0],(x,y)))<=self.ob[1])):
            return False
        
        #Concludes geometry tests!
        return True

    def convToWindow(self,(x,y)):
        newx = self.winMaxX*x/2.
        newy = self.winMaxY*(1.-y)/2.
        return (int(newx),int(newy))
 
    def run(self):
        r = rospy.Rate(10)
        # continue until quit
        #radiusConversion = distance(self.winMaxX,self.winMaxY)/distance(a1,a2)
        (otx,oty) = self.convToWindow(self.ot[0])
        (obx,oby) = self.convToWindow(self.ob[0])
        (itx,ity) = self.convToWindow(self.it[0])
        (ibx,iby) = self.convToWindow(self.ib[0])
        while not rospy.is_shutdown():
            self.update_window_control()
            self.background.fill((255,255,255,0))
            #Print actual point
            #(x,y) = (1.*(float(self.winX))/self.winMaxX,
            #    1.-2*self.winY/float(self.winMaxY))
            (x,y) = self.convToWindow((self.settings.x,self.settings.y))
            pygame.draw.rect(self.background,(0,0,0),
            pygame.Rect(x-5,y-5,10,10))
            #Draw circle bounds
            pygame.draw.ellipse(self.background,(0,0,0),
                pygame.Rect(otx-1*int(self.winMaxX*self.ot[1]/2.),
                            oty-1*int(self.winMaxY*self.ot[1]/2.),
                            2*int(self.winMaxX*self.ot[1]/2.),
                            2*int(self.winMaxY*self.ot[1]/2.)),2) 
            pygame.draw.ellipse(self.background,(0,0,0),
                pygame.Rect(obx-1*int(self.winMaxX*self.ob[1]/2.),
                            oby-1*int(self.winMaxY*self.ob[1]/2.),
                            2*int(self.winMaxX*self.ob[1]/2.),
                            2*int(self.winMaxY*self.ob[1]/2.)),2)   
            pygame.draw.ellipse(self.background,(0,0,0),
                pygame.Rect(itx-1*int(self.winMaxX*self.it[1]/2.),
                            ity-1*int(self.winMaxY*self.it[1]/2.),
                            2*int(self.winMaxX*self.it[1]/2.),
                            2*int(self.winMaxY*self.it[1]/2.)),2)             
            pygame.draw.ellipse(self.background,(0,0,0),
                pygame.Rect(ibx-1*int(self.winMaxX*self.ib[1]/2.),
                            iby-1*int(self.winMaxY*self.ib[1]/2.),
                            2*int(self.winMaxX*self.ib[1]/2.),
                            2*int(self.winMaxY*self.ib[1]/2.)),2)
            # publish to topic
            self.pubArm.publish(self.angles)
            verbose = rospy.get_param("~verbose", False)
            if verbose:
                rospy.loginfo(self.angles)  # next iteration
        r.sleep()


#distance between points
def ddistance((x1,y1),(x2,y2)):
    return distance((x2-x1),(y2-y1))

# makes angle between -pi and pi
def modAngle(x):
    # if already good
    if x >= -pi and x <= pi:
        return x
    # if not, modulate x value
    return x - 2 * pi * round(x / 2. / pi)

def sgn(x):
    if x == 0:
        return 1
    return x / abs(x)


# gets the euclidian distance to a point from the origin
def distance(x, y, z=0):
    return sqrt(x ** 2 + y ** 2 + z ** 2)

# arctangent function that adjusts based on
# what quadrant it should be in

# function will not return an angle 
# greater in magnitude than pi
# models ArcTan(y/x)
def ArcTan(x, y):
    if x == 0:
        if y > 0:
            return pi / 2
        return 3 * pi / 2
    initial = atan(y / x)
    if initial >= 0:
        if y >= 0:  # correct quadrant (1)
            return initial
        # incorrect (3)
        return initial - pi
    else:
        if y < 0:  # correct quadrant (4)
            return initial
        # incorrect (2) 
        return initial + pi


# Some basic vector geometry functions:
def dot(u, v):
    total = 0
    for x, y in zip(u, v):
        total += x * y
    return total


def project(u, v):
    mag = 0
    for x in v:
        mag += x ** 2
    scalar = dot(u, v) / mag
    result = []
    for x in v:
        result.append(x * scalar)
    return result


def reflect(u, v):
    projection = project(u, v)
    result = []
    for x, y in zip(u, projection):
        result.append(2 * y - x)
    return result


# feed in cartesian coordinates and this function will convert it to
# our polar/cartesian mix
def convertCartesian(x, y, z):
    return (distance(x, z), y, ArcTan(x, z))


# function will give two sets of angles for the robotic arm (both valid)
# , when told which point in the xy plane needs to be reached
def possibleAngles(x, y):
    angles = [[0., 0.], [0., 0.]]

    # the following are the equations for each angle, computed by mathematica

    # variables to reduce reduntant calculation
    preCalculated1 = a1 ** 2 - a2 ** 2 + x ** 2 + y ** 2
    preCalculated2 = sqrt(-a1 ** 4 - (-a2 ** 2 + x ** 2 + y ** 2) ** 2 + 2 * a1 ** 2 * (a2 ** 2 + x ** 2 + y ** 2))
    # real nasty equations to get angles
    angles[0][0] = ArcTan(
        x * preCalculated1 - y * preCalculated2 / sgn(y),
        y * preCalculated1 + x * preCalculated2 / sgn(y))

    angles[0][1] = -ArcTan(-2 * a1 ** 2 + preCalculated1, -preCalculated2 / sgn(y))

    # other set of angles are a reflection in the direction vector to the point
    angles[1][0] = ArcTan(
        x * (preCalculated1) + (y * preCalculated2) / sgn(y),
        y * (preCalculated1) - (x * preCalculated2) / sgn(y))
    # simple calculation for opposing angle on forearm :)
    angles[1][1] = -angles[0][1]
    return angles


# get next reasonable angles for the arm
# will return angles, and boolean to say
# whether arm should stop or not
def nextAngle(initialAngles, x, y):
    # quick check for safety
    if a1 + a2 < distance(x, y):
        # point is unreachable;
        # don't calculate anything!
        return [[0, 0], False]

    # get possible angles to move to
    (angleset1, angleset2) = possibleAngles(x, y)

    angleset1[0] = modAngle(angleset1[0])
    angleset1[1] = modAngle(angleset1[1])
    angleset2[0] = modAngle(angleset2[0])
    angleset2[1] = modAngle(angleset2[1])

    # both sets are not good until tested
    set1good, set2good = False, False

    # test if angles are in fact out of bounds
    if uppMin <= angleset1[0] <= uppMax:
        if forMin <= angleset1[1] <= forMax:
            # if all angles in set are fine, declare the set okay
            set1good = True

    if uppMin <= angleset2[0] <= uppMax:
        if forMin <= angleset2[1] <= forMax:
            set2good = True

        # if both sets are out of bounds,
    if not set1good and not set2good:
        # return false for movement
        return [[0, 0], False]
    # if only one is good, return that one
    elif set1good and not set2good:
        return [angleset1, True]
    elif set2good and not set1good:
        return [angleset2, True]

    # if both are good, then find best angleset for current position
    # get total angle deviation for each angle set
    deviation1 = abs(initialAngles[0] - angleset1[0])
    deviation1 += abs(initialAngles[1] - angleset1[1])
    deviation2 = abs(initialAngles[0] - angleset2[0])
    deviation2 += abs(initialAngles[1] - angleset2[1])

    # if first set is closer to current position, return it
    if deviation1 <= deviation2:
        return [angleset1, True]
    return [angleset2, True]

if __name__ == '__main__':
    print "Initializing Node"
    reader1 = ArmControlReader()
    print "Running Node"
    reader1.run()
    rospy.spin()
    f = open('data.csv','w')
    for x in masterPoints:
        f.write(str(x))

