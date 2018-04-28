from numpy import sin, cos
from numpy.random import normal, random
from scipy.stats import norm
import numpy as np
import MapBuilder as mb
import math

class Particle:
    a1 = 0.01
    a2 = 0.01
    a3 = 0.01
    a4 = 0.01

    fidelity = 0.1  # this is the map fidelity(e.g 0.1 means every pixel is 10cm * 10cm)
    genX = 0.0  # this is the ros prediction for its pos
    genY = 0.0
    genTH = 0.0
    NoP = 1 # Number of Particles
    hitmap = None
    sizeX = 101
    sizeY = 111

    def __init__(self, x, y, th, grid, prop, prop_map, tick_map):
        self.x = x
        self.y = y
        self.th = th
        self.prop = prop
        self.grid = grid.copy()
        self.propMap = prop_map.copy()
        self.tickMap = tick_map.copy()

    def move_particle(self, drot1, dtrans, drot2):
        a1 = Particle.a1
        a2 = Particle.a2
        a3 = Particle.a3
        a4 = Particle.a4

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

    # The function must have in

    def merg_map_maker(self, func, start_angle, end_angle, angle_incr, ranges, max_depth):
        merger_map = func(self.x, self.y, self.th, start_angle, end_angle,
                          angle_incr, ranges, max_depth, Particle.sizeX, Particle.sizeY,Particle.fidelity)
        mb.print_map(merger_map)
        return merger_map

def initParticles():
    Particles = list()
    x = 0
    y = 0
    th = 0
    grid = np.ndarray(shape=(Particle.sizeX,Particle.sizeY), dtype=np.int)
    grid.fill(-1)
    prop_map = np.ndarray(shape=(Particle.sizeX,Particle.sizeY), dtype=np.float)
    prop_map.fill(-1.0)
    tick_map = np.ndarray(shape=(Particle.sizeX, Particle.sizeY), dtype=np.int)
    tick_map.fill(0)
    for i in range(0, Particle.NoP):
        Particles.append(Particle(x, y, th, grid, 1, prop_map, tick_map))
    return Particles

def mapUpdate(func,Particles, StartAngle, EndAngle, Dangle, Points, MaxDepth):
    count = 0
    for p in Particles:
        count+=1
        newMap = p.merg_map_maker(func,StartAngle, EndAngle, Dangle, Points, MaxDepth)
    return newMap

def coord_to_grid_coord(x, y):

    x *= 2 / Particle.fidelity
    y *= 2 / Particle.fidelity

    x += (Particle.sizeX - 1) / 2
    y += (Particle.sizeY - 1) / 2

    return int(math.floor(x)), int(math.floor(y))
