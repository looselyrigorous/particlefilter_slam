from numpy.random import normal
from numpy import abs
from numpy import sin,cos
import math
from scipy.stats import norm

from math import atan2
a1 = 0.01
a2 = 0.01
a3 = 0.01
a4 = 0.01

class Particle:
    fidelity = 0.1 ###this is the map fidelity(e.g 0.1 means every pixel is 10cm * 10cm)
    size = 51  ### the size of the map (map always is square for now)
    genX = 0.0 ### this is the ros prediction for its pos
    genY = 0.0
    genTH = 0.0
    NoP = 256 ### Number of Particles

    def __init__(self, x, y, th, map, prop, propMap,tickMap):
        self.x = x
        self.y = y
        self.th = th
        self.map = map ### The map as calculated via the propMap (if propability >=0.5 then pixel is black if propability <0.5 pixel is white else gray)
        self.prop = prop ### A number that is the propability of this Particle to be perfectly in line with our external model of space reality
        self.propMap = propMap ##This is the map that holds the propabilities of every square mathematically [0,1]
        self.tickMap = tickMap ##TickMap is a map that shows how many times have each square seperately being *seen*



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
                    sum+= self.calcSqrError(newMap,i,j)

        self.prop *= 1/sum

    def calcSqrError(self,newMap,x0,y0):
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

    def calculateDs(X, Y, TH):
        drot1 = atan2(Y - Particle.genY, X - Particle.genX) - Particle.genTH
        dtrans = math.sqrt((Particle.genX - X)**2 + (Particle.genY - Y)**2)
        drot2 = TH - Particle.genTH - drot1

        return drot1,dtrans,drot2


    def replaceGenDims(X, Y, TH):
        Particle.genX = X
        Particle.genY = Y
        Particle.genTH = TH

    def newMapMaker(self,StartAngle,EndAngle,Dangle,Points):
        newMap = [[-1] * Particle.size for i in range(Particle.size)]

    def realCoordToGrid(self,x,y):
        middlePoint = ((Particle.size-1)/2)*Particle.fidelity
        return ((size-1)/2)*(x-middlePoint),((size-1)/2)*(y-middlePoint)

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
