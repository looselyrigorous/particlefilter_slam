from numpy.random import normal,random

from numpy import abs
from numpy import sin,cos
import math
from scipy.stats import norm
import numpy as np
from copy import deepcopy


from math import atan2
a1 = 0.01
a2 = 0.01
a3 = 0.01
a4 = 0.01

class Particle:
    fidelity = 0.1 ###this is the map fidelity(e.g 0.1 means every pixel is 10cm * 10cm)
    size = 101  ### the size of the map (map always is square for now)
    genX = 0.0 ### this is the ros prediction for its pos
    genY = 0.0
    genTH = 0.0
    NoP = 256 ### Number of Particles

    def __init__(self, x, y, th, map, propMap,tickMap):
        self.x = x
        self.y = y
        self.th = th
        self.map = deepcopy(map) ### The map as calculated via the propMap (if propability >=0.5 then pixel is black if propability <0.5 pixel is white else gray)
        self.prop = 1 ### A number that is the propability of this Particle to be perfectly in line with our external model of space reality
        self.propMap = deepcopy(propMap) ##This is the map that holds the propabilities of every square mathematically [0,1]
        self.tickMap = deepcopy(tickMap) ##TickMap is a map that shows how many times have each square seperately being *seen*


    ##Calculates particle position and propability of that position

    def moveParticle(self, drot1, dtrans, drot2):

        sgm1 = a1*drot1 + a2*dtrans
        sgm2 = a3*dtrans + a4*(drot2+drot1)
        sgm3 = a1*drot2 + a2*dtrans

        dev1 = normal(0,sgm1)
        dev2 = normal(0,sgm2)
        dev3 = normal(0,sgm3)

        drote1 = drot1 + dev1
        dtranse = dtrans + dev2
        drote2 = drot2 + dev3

        self.x = x + dtranse*cos(self.th + drote1)
        self.y = y + dtranse*sin(self.th + drote1)
        self.th = self.th + drote1 + drote2

        self.prop = norm(0,sgm1).pdf(dev1)*norm(0,sgm2).pdf(dev2)*norm(0,sgm3).pdf(dev3)

    #Calculates the newPropabilityMap

    def propabilityMapMaker(self,newMap):
        for i in range(0,Particle.size):
            for j in range(0,Particle.size):
                if newMap[i][j] != -1:
                    self.tilePropability(i,j,newMap[i][j])
    #Calculates the propability of each pixel in the map

    def tilePropability(self,x,y,number):
        if self.propMap[x][y] == -1:
            self.propMap = number
            self.tickMap[x][y] = 1
            return

        self.propMap[x][y] *= self.tickMap[x][y]
        self.propMap[x][y] += number
        self.tickMap[x][y] += 1
        self.propMap[x][y] /= self.tickMap[x][y]

    #Calculates the new Map

    def mapMaker(self):
        for i in range(0,Particle.size):
            for j in range(0,Particle.size):
                TileMaker(i,j,self.propMap[i][j])


    def TileMaker(self,x,y,prop):
        self.map[x][y] = math.round(prop)

    #Calculates the absolute error of the Map and the newMap

    def calcErrorMap(self,newMap):
        sum = 0
        for i in range(0,Particle.size):
            for j in range(0, Particle.size):
                if newMap[i][j] != -1:
                    sum+= self.calcError(newMap, i, j)

        self.prop *= 1/sum
    ##This might need to become square Error of each different pixel

    def calcError(self, newMap, x0, y0):
        for i in range(0,2*Particle.size):
            if self.calcErrorRadius(newMap, x0, y0, i) == True:
                return i

        return i

    ## Calculating if there exist any same tiles in a radius and then returns True or False
    ## It uses a modified draw Circle algorithm

    def calcErrorRadius(self, newMap, x0, y0, radius):
        x = 0
        y = 0
        dx = 1
        dy = 1
        err = dx - (radius*2)

        count = 0

        while x>=y:
            count += self.checkSimilarity(newMap[x0][y0], x0 + x, y0 + y)
            count += self.checkSimilarity(newMap[x0][y0], x0 + x, y0 - y)
            count += self.checkSimilarity(newMap[x0][y0], x0 - x, y0 + y)
            count += self.checkSimilarity(newMap[x0][y0], x0 - x, y0 - y)
            count += self.checkSimilarity(newMap[x0][y0], x0 + y, y0 + x)
            count += self.checkSimilarity(newMap[x0][y0], x0 + y, y0 - x)
            count += self.checkSimilarity(newMap[x0][y0], x0 - y, y0 + x)
            count += self.checkSimilarity(newMap[x0][y0], x0 - y, y0 - x)

            if count>0:
                return True
            if err <= 0:
                y+=1
                err +=dy
                dy +=2

            if err>0:
                x-=1
                dx +=2
                err += dx -(radius*2)

        return False

    def checkSimilarity(self,checkPoint,x,y):
        if x<Particle.size and x>=0 and y<Particle.size and y>=0:
            if checkPoint == self.map[x][y]:
                return 1
            else:
                return 0
        return 0

    def newMapMaker(self, StartAngle, EndAngle, Dangle, Points, MaxDepth):
        newMap = [[-1] * Particle.size for i in range(Particle.size)]
        x0 = self.x
        y0 = self.y
        StartAngle += self.th
        EndAngle += self.th
        count = 0

        for i in range(StartAngle,EndAngle,Dangle):
            d = Points[count]
            if d!=np.inf and d!=np.nan:
                x1 = x0 + x0 + d * np.cos(i)
                y1 = y0 + d * np.sin(i)
                Wall = True
            else:### we need to know what to do here
                x1 = x0 + x0 + MaxDepth * np.cos(i)
                y1 = y0 + MaxDepth * np.sin(i)
                Wall = False
            count+=1

            xS,yS = realCoordToGrid(x0,y0)
            xE,yE = realCoordToGrid(x1,y1)
            newMap = newTiles(newMap,xS,yS,xE,yE,Wall)

        return newMap

    def newTiles(self,newMap,x0,y0,x1,y1,Wall):
        deltax = x1 - x0
        deltay = y1 - y0
        deltaerr = math.abs(deltay / deltax)

        error = 0.0
        y = y0

        for x in range(x0, x1):
            newMap[x][y] = 0
            error += deltaerr
            while error >= 0.5:
                y += np.sign(deltay)
                error -= 1.0
        if Wall == True:
            newMap[x1][y1] = 1

        return newMap

    def normalize(self,ammount):
        self.prop /= ammount

    def lineUp(self,start):
        self.lineStart = start
        self.lineEnd = start + self.prop

    def survive(self,number):
        if number>=self.lineStart and number<self.lineEnd:
            return True
        return False

    def procreate(self):
        return Particle(self.x,self.y,self.th,self.map,self.propMap,self.tickMap)

def calculateDs(X, Y, TH):
    drot1 = atan2(Y - Particle.genY, X - Particle.genX) - Particle.genTH
    dtrans = math.sqrt((Particle.genX - X)**2 + (Particle.genY - Y)**2)
    drot2 = TH - Particle.genTH - drot1

    return drot1,dtrans,drot2


def replaceGenDims(X, Y, TH):
    Particle.genX = X
    Particle.genY = Y
    Particle.genTH = TH


def realCoordToGrid(x,y):
    x *= 2/Particle.fidelity
    y *= 2/Particle.fidelity

    x += (Particle.size-1)/2
    y += (Particle.size-1)/2

    if x<0:
        x = 0
    elif x> Particle.size:
        x = Particle.size
    if y<0:
        y = 0
    elif y>Particle.size:
        y = Particle.size

    return x,y

#This function normalizes the propability of all the Particles to add to 1
#Then every particle has a starting number and an ending number that we must hit in order for it to survive


def normalizeAndLineUp(Particles):
    sum =0
    for p in Particles:
        sum+=p.prop
    for p in Particles:
        p.normalize(sum)
    nextStartPoint = 0
    for p in Particles:
        p.lineUp(sum)

def selectSurvivors(Particles):
    step = 1/NoP
    startPoint = random()/Particle.NoP
    survivors = list()
    particleIndex = 0
    currParticle = Particles[particleIndex]
    currPoint = startPoint

    for i in range(0,Particle.NoP):
        survive = currParticle.survive(currPoint)
        if survive:
            survivors.append(currParticle)
            currPoint+=step
        else:
            particleIndex+=1
            currParticle = Particles[particleIndex]
            i-=1

    children = list()

    for s in survivors:
        children.append(s.procreate())

    return children


def quaternion_to_euler_angle(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z


