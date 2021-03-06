from numpy.random import normal, random
from numpy import abs
from numpy import sin, cos
import math
from scipy.stats import norm
import numpy as np
from copy import deepcopy
from math import atan2
import MapBuilder

a1 = 0.01
a2 = 0.01
a3 = 0.01
a4 = 0.01

count = 0
class Particle:
    fidelity = 0.1  ###this is the map fidelity(e.g 0.1 means every pixel is 10cm * 10cm)
    genX = 0.0  ### this is the ros prediction for its pos
    genY = 0.0
    genTH = 0.0
    NoP = 1  ### Number of Particles
    hitmap = None
    size = 101

    def __init__(self, x, y, th, map, prop, propMap, tickMap):
        self.x = x
        self.y = y
        self.th = th
        self.map = map.copy()  ### The map as calculated via the propMap (if propability >=0.5 then pixel is black if propability <0.5 pixel is white else gray)
        self.prop = prop  ### A number that is the propability of this Particle to be perfectly in line with our external model of space reality
        self.propMap = propMap.copy()  ##This is the map that holds the propabilities of every square mathematically [0,1]
        self.tickMap = tickMap.copy()  ##TickMap is a map that shows how many times have each square seperately being *seen*

    ##Calculates particle position and propability of that position

    def moveParticle(self, drot1, dtrans, drot2):

        sgm1 = abs(a1 * drot1 + a2 * dtrans)
        sgm2 = abs(a3 * dtrans + a4 * (drot2 + drot1))
        sgm3 = abs(a1 * drot2 + a2 * dtrans)

        dev1 = normal(0, sgm1)
        dev2 = normal(0, sgm2)
        dev3 = normal(0, sgm3)

        drote1 = drot1 + dev1
        dtranse = dtrans + dev2
        drote2 = drot2 + dev3

        self.x += dtranse * cos(self.th + drote1)
        self.y += dtranse * sin(self.th + drote1)
        self.th += drote1 + drote2

        self.prop = norm(0, sgm1).pdf(dev1) * norm(0, sgm2).pdf(dev2) * norm(0, sgm3).pdf(dev3)

    # Calculates the newPropabilityMap

    def propabilityMapMaker(self, newMap):
        for i in range(0, Particle.size):
            for j in range(0, Particle.size):
                if newMap[i, j] != -1:
                    self.tilePropability(i, j, newMap[i, j])

    # Calculates the propability of each pixel in the map

    def tilePropability(self, x, y, number):
        if self.propMap[x, y] == -1:
            self.propMap[x, y] = number
            self.tickMap[x, y] = 1
            return

        self.propMap[x, y] *= self.tickMap[x, y]
        self.propMap[x, y] += number
        self.tickMap[x, y] += 1
        self.propMap[x, y] /= self.tickMap[x, y]

    # Calculates the new Map

    def mapMaker(self):
        for i in range(0, Particle.size):
            for j in range(0, Particle.size):
                self.TileMaker(i, j, self.propMap[i, j])

    def TileMaker(self, x, y, prop):
        self.map[x, y] = round(prop)

    # Calculates the absolute error of the Map and the newMap

    def calcErrorMap(self, newMap):
        sum = 0
        for i in range(0, Particle.size):
            for j in range(0, Particle.size):
                if newMap[i, j] != -1:
                    sum += self.calcError(newMap, i, j)

        self.prop *= 1 / (sum + 1)

    ##This might need to become square Error of each different pixel

    def calcError(self, newMap, x0, y0):
        for i in range(0, 2 * Particle.size):
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
        err = dx - (radius * 2)

        count = 0

        while x >= y:
            count += self.checkSimilarity(newMap[x0, y0], x0 + x, y0 + y)
            count += self.checkSimilarity(newMap[x0, y0], x0 + x, y0 - y)
            count += self.checkSimilarity(newMap[x0, y0], x0 - x, y0 + y)
            count += self.checkSimilarity(newMap[x0, y0], x0 - x, y0 - y)
            count += self.checkSimilarity(newMap[x0, y0], x0 + y, y0 + x)
            count += self.checkSimilarity(newMap[x0, y0], x0 + y, y0 - x)
            count += self.checkSimilarity(newMap[x0, y0], x0 - y, y0 + x)
            count += self.checkSimilarity(newMap[x0, y0], x0 - y, y0 - x)

            if count > 0:
                return True
            if err <= 0:
                y += 1
                err += dy
                dy += 2

            if err > 0:
                x -= 1
                dx += 2
                err += dx - (radius * 2)

        return False

    def checkSimilarity(self, checkPoint, x, y):
        if x < Particle.size and x >= 0 and y < Particle.size and y >= 0:
            if checkPoint == self.map[x, y]:
                return 1
            else:
                return 0
        return 0

    def newMapMaker(self, StartAngle, EndAngle, Dangle, Points, MaxDepth):
        newMap = np.ndarray(shape=(Particle.size, Particle.size), dtype=float)
        newMap.fill(-1)
        x0 = self.x
        y0 = self.y
        StartAngle += self.th
        EndAngle += self.th
        count = 0
        maxCount = len(Points)
        Dangle = (EndAngle - StartAngle) / maxCount
        for count in range(0, maxCount):
            i = StartAngle + (Dangle * count)
            d = Points[count]
            x1 = 0
            y1 = 0
            if d != np.inf and not math.isnan(d):
                x1 = x0 + d * np.cos(i)
                y1 = y0 + d * np.sin(i)
                Wall = True
            else:
                x1 = x0 + MaxDepth * np.cos(i)
                y1 = y0 + MaxDepth * np.sin(i)
                Wall = False

            xS, yS = realCoordToGrid(x0, y0)
            xE, yE = realCoordToGrid(x1, y1)
            newMap = self.newTiles(newMap, xS, yS, xE, yE, Wall)

        return newMap

    def newTiles(self, newMap, x0, y0, x1, y1, Wall):
        if x0 == x1:
            if y0 == y1:
                return newMap
            else:
                newMap = plotLineY(newMap, y0, y1, x1)

        elif y0 == y1:
            newMap = plotLineX(newMap, x0, x1, y1)

        elif (abs(x0 - x1) > abs(y0 - y1)):
            newMap = plotLineXHigh(newMap, x0, x1, y0, y1)

        else:
            newMap = plotLineYHigh(newMap, x0, x1, y0, y1)

        if Wall == True and x1 > 0 and x1 < Particle.size and y1 > 0 and y1 < Particle.size:
            newMap[x1, y1] = 1

        return newMap

    def normalize(self, ammount):
        self.prop /= ammount

    def lineUp(self, start):
        self.lineStart = start
        self.lineEnd = start + self.prop
        return lineEnd

    def survive(self, number):
        if number >= self.lineStart and number < self.lineEnd:
            return True
        return False

    def procreate(self):
        return Particle(self.x, self.y, self.th, self.map, self.prop, self.propMap, self.tickMap)


def calculateDs(X, Y, TH):
    drot1 = atan2(Y - Particle.genY, X - Particle.genX) - Particle.genTH
    dtrans = math.sqrt((Particle.genX - X) ** 2 + (Particle.genY - Y) ** 2)
    drot2 = TH - Particle.genTH - drot1

    return drot1, dtrans, drot2


def replaceGenDims(X, Y, TH):
    Particle.genX = X
    Particle.genY = Y
    Particle.genTH = TH


def initParticles():
    Particles = list()
    x = 0
    y = 0
    th = 0
    map = np.ndarray(shape=(Particle.size, Particle.size), dtype=np.int)
    map.fill(-1)
    propMap = np.ndarray(shape=(Particle.size, Particle.size), dtype=np.float)
    propMap.fill(-1.0)
    tickMap = np.ndarray(shape=(Particle.size, Particle.size), dtype=np.int)
    tickMap.fill(0)
    for i in range(0, Particle.NoP):
        Particles.append(Particle(x, y, th, map, 1, propMap, tickMap))
    return Particles


def odomUpdate(Particles, X, Y, TH):
    drot1, dtrans, drot2 = calculateDs(X, Y, TH)

    for i in Particles:
        i.moveParticle(drot1, dtrans, drot2)

    replaceGenDims(X, Y, TH)


def mapUpdate(Particles, StartAngle, EndAngle, Dangle, Points, MaxDepth):
    count = 0
    for p in Particles:
        count += 1
        newMap = p.newMapMaker(StartAngle, EndAngle, Dangle, Points, MaxDepth)
        p.calcErrorMap(newMap)
        p.propabilityMapMaker(newMap)
        p.mapMaker()
        printMap(newMap)


# The hitmap is made for kobuki that is about 1 meter in diameter so it doesnt hit its fat body when roaming :)

def hitMapUpdate(Particles):
    maxProp = 0
    bestParticle = None
    for p in Particles:
        if p.prop > maxProp:
            bestParticle = p
            maxProp = p.prop

    hitmap = np.ndarray(shape=(Particle.size, Particle.size), dtype=int)
    hitmap.fill(-1)
    map = bestParticle.map
    for i in range(0, Particle.size):
        for j in range(0, Particle.size):
            tile = map[i, j]
            if tile == 1:
                for k in range(0, 5):
                    for l in range(0, 5):
                        x = i + k
                        y = j + l
                        if x >= 0 and x < Particle.size and y >= 0 and y < Particle.size:
                            hitmap[x, y] = 1

    Particle.hitmap = hitmap


# We need a function that makes the robot roam in conjuction with its velocity and such things


# It transforms real coordinates to grid coordinates
def realCoordToGrid(x, y):
    x *= 2 / Particle.fidelity
    y *= 2 / Particle.fidelity

    x += (Particle.size - 1) / 2
    y += (Particle.size - 1) / 2

    return int(math.floor(x)), int(math.floor(y))


# This function normalizes the propability of all the Particles to add to 1
# Then every particle has a starting number and an ending number that we must hit in order for it to survive


def normalizeAndLineUp(Particles):
    sum = 0
    for p in Particles:
        sum += p.prop
    for p in Particles:
        p.normalize(sum)
    nextStartPoint = 0
    for p in Particles:
        nextStartPoint += p.line_up(nextStartPoint)


def selectSurvivors(Particles):
    step = 1 / NoP
    startPoint = random() / Particle.NoP
    survivors = list()
    particleIndex = 0
    currParticle = Particles[particleIndex]
    currPoint = startPoint

    for i in range(0, Particle.NoP):
        survive = currParticle.survive(currPoint)
        if survive:
            survivors.append(currParticle)
            currPoint += step
        else:
            particleIndex += 1
            currParticle = Particles[particleIndex]
            i -= 1

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


def printBestMap(Particles):
    maxProp = -1
    bestParticle = None
    for p in Particles:
        if p.prop > maxProp:
            bestParticle = p
            maxProp = p.prop
    map = bestParticle.map
    printMap(map)


def printMap(Map):
    for i in range(0, Particle.size):
        stringer = ''
        for j in range(0, Particle.size):
            occ = Map[i, j]
            if occ == 1:
                stringer += 'x'
            elif occ == 0:
                stringer += 'o'
            else:
                stringer += ' '
        print(stringer)


# These are my own line function to check 

def plotLineX(newMap, x0, x1, y):
    for x in range(x0, x1, np.sign(x1 - x0)):
        if x < 0 or x >= Particle.size:
            return newMap
        newMap[x, y] = 0
    return newMap


def plotLineY(newMap, y0, y1, x):
    for y in range(y0, y1, np.sign(y1 - y0)):
        if y < 0 or y >= Particle.size:
            return newMap
        newMap[x, y] = 0
    return newMap


def plotLineXHigh(newMap, x0, x1, y0, y1):
    error = abs(float(y0 - y1) / float(x0 - x1))
    Yerror = 0
    y = y0
    for x in range(x0, x1, np.sign(x1 - x0)):
        if x < 0 or x >= Particle.size:
            return newMap
        newMap[x, y] = 0
        Yerror += error
        if Yerror >= 1:
            y += np.sign(y1 - y0)
            if y < 0 or y >= Particle.size:
                return newMap
            newMap[x, y] = 0
            Yerror -= 1

    return newMap


def plotLineYHigh(newMap, x0, x1, y0, y1):
    error = abs(float(x0 - x1) / float(y0 - y1))
    Xerror = 0
    x = x0
    for y in range(y0, y1, np.sign(y1 - y0)):
        if y < 0 or y >= Particle.size:
            return newMap
        newMap[x, y] = 0
        Xerror += error
        if Xerror >= 1:
            x += np.sign(x1 - x0)
            if x < 0 or x >= Particle.size:
                return newMap
            newMap[x, y] = 0
            Xerror -= 1

    return newMap
