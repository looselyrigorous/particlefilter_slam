from numpy import sin, cos
from numpy.random import normal, random
from scipy.stats import norm
import numpy as np
import MapBuilder as mb
import math
from math import atan2
import time
import sys
#import Preprocess as pp
from PIL import Image, ImageDraw, ImageFont



class Particle:
    a1 = 0.00000000001
    a2 = 0.00000000001
    a3 = 0.0000000001
    a4 = 0.0000000001
    count = 0

    fidelity = 0.1  # this is the map fidelity(e.g 0.1 means every pixel is 10cm * 10cm)
    genX = 0.0  # this is the ros prediction for its pos
    genY = 0.0
    genTH = 0.0
    NoP = 10  # Number of Particles
    hitmap = None
    sizeX = 301
    sizeY = 301



    def __init__(self, x, y, th, grid, prop, prop_map, tick_map):
        self.x = x
        self.y = y
        self.th = th
        self.prop = prop
        self.grid = grid.copy()
        self.prop_map = prop_map.copy()
        self.tick_map = tick_map.copy()
        self.line_start = 0
        self.line_end = 0

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
        self.th += (drote1 + drote2)
        self.th = self.th

        print(self.x,self.y)

        normalizing_factor = (norm(0, sgm1).pdf(0)) * (norm(0, sgm2).pdf(0)) * (norm(0, sgm3).pdf(0))
        weight = (norm(0, sgm1).pdf(dev1)) * (norm(0, sgm2).pdf(dev2)) * (norm(0, sgm3).pdf(dev3))
        #print(normalizing_factor,weight)
        self.prop *= weight/normalizing_factor


    # The function must have in

    def print_map(self):
        if Particle.count == 0:
            mb.print_map(self.grid)
            self.make_photo()
        Particle.count += 1
        if Particle.count == 201:
            Particle.count = 0

    def make_photo(self):

        myarray = np.ndarray(shape = (Particle.sizeX,Particle.sizeY))
        for i in range(Particle.sizeX):
            for j in range(Particle.sizeY):
                if self.grid[i,j] == -1:
                    myarray[i,j] = 0.5
                else:
                    myarray[i,j] = 1 - self.grid[i,j]
        Photo = Image.fromarray(np.uint8((myarray)*255))
        #Photo.show()
        Photo.save('result.bmp')



    def merge_map_maker(self, func, start_angle, end_angle, angle_incr, ranges, max_depth):
        merger_map = func(self.x, self.y, self.th, start_angle, end_angle,
                          angle_incr, ranges, max_depth, Particle.sizeX, Particle.sizeY, Particle.fidelity)


        return merger_map

    def prop_map_maker(self, func, merger_map):
        self.prop_map = func(self.prop_map, self.tick_map, merger_map)



    def grid_maker(self, func):
        self.grid = func(self.grid,self.prop_map)

    def map_error(self,func,merger_map):

        new_prop = func(merger_map,self.grid,self.prop_map,self.tick_map)
        self.prop *= new_prop


    def normalize(self, ammount):
        self.prop /= ammount

    def line_up(self, start):
        self.line_start = start
        self.line_end = start + self.prop
        return self.line_end

    def survive(self, number):
        if self.line_start <= number < self.line_end:
            return True
        return False

    def procreate(self):
        return Particle(self.x, self.y, self.th, self.grid, self.prop, self.prop_map, self.tick_map)




def init_particles(x,y,th):
    Particles = list()
    replace_gen_pos(x,y,th)
    grid = np.ndarray(shape=(Particle.sizeX, Particle.sizeY), dtype=np.int)
    grid.fill(-1)
    prop_map = np.ndarray(shape=(Particle.sizeX, Particle.sizeY), dtype=np.float)
    prop_map.fill(-1.0)
    tick_map = np.ndarray(shape=(Particle.sizeX, Particle.sizeY), dtype=np.int)
    tick_map.fill(0)
    for i in range(0, Particle.NoP):
        Particles.append(Particle(0, 0, 0, grid, 1.0, prop_map, tick_map))
    return Particles


def map_update(merge_map_func, prop_map_func, grid_make_func, map_error_func,
               Particles, start_angle, end_angle, angle_incr, ranges, max_depth):
    count = 0
    for p in Particles:
        count += 1
        merger_map = p.merge_map_maker(merge_map_func, start_angle, end_angle, angle_incr, ranges, max_depth)
        p.map_error(map_error_func,merger_map)
        p.prop_map_maker(prop_map_func, merger_map)
        p.grid_maker(grid_make_func)
    return merger_map



def odom_update(Particles, X, Y, TH):
    drot1, dtrans, drot2 = calculate_diff(X, Y, TH)

    for i in Particles:
        i.move_particle(drot1, dtrans, drot2)
    replace_gen_pos(X, Y, TH)


def replace_gen_pos(X, Y, TH):
    Particle.genX = X
    Particle.genY = Y
    Particle.genTH = TH

def sum_weights(Particles):
    sum = 0
    for p in Particles:
        sum += p.prop
    return sum

def normalize_line_up(Particles):
    sum = sum_weights(Particles)
    sys.stdout.flush()
    for p in Particles:
        p.normalize(sum)
    nextStartPoint = 0
    for p in Particles:
        nextStartPoint = p.line_up(nextStartPoint)


def select_survivors(Particles):

    normalize_line_up(Particles)
    weight_sum = sum_weights(Particles)
    step = 1 / Particle.NoP
    startPoint = random() / Particle.NoP
    survivors = list()
    particleIndex = 0
    currParticle = Particles[particleIndex]
    currPoint = startPoint

    for i in range(0, Particle.NoP):
        survive = currParticle.survive(currPoint)
        currPoint = startPoint + i*step
        if survive:
            survivors.append(currParticle)
        else:
            particleIndex += 1
            if particleIndex >= Particle.NoP:
                print('ERROR',particleIndex,currPoint,currParticle.prop)
                sys.stdout.flush()
                exit()
            currParticle = Particles[particleIndex]
            i -= 1

    children = list()

    for s in survivors:
        children.append(s.procreate())

    #print_best_particle(children)

    return children


def calculate_diff(X, Y, TH):
    drot1 = atan2(Y - Particle.genY, X - Particle.genX) - Particle.genTH
    dtrans = math.sqrt((Particle.genX - X) ** 2 + (Particle.genY - Y) ** 2)
    drot2 = (TH - Particle.genTH - drot1)
    return drot1, dtrans, drot2


def coord_to_grid_coord(x, y):
    x *= 2 / Particle.fidelity
    y *= 2 / Particle.fidelity

    x += (Particle.sizeX - 1) / 2
    y += (Particle.sizeY - 1) / 2

    return int(math.floor(x)), int(math.floor(y))


def quaternion_to_radians(w, x, y, z):
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

    return np.deg2rad(X), np.deg2rad(Y), np.deg2rad(Z)


def print_best_particle(Particles):
    max = 0
    best_particle = None
    for i in Particles:
        if max < i.prop:
            max = i.prop
            best_particle = i
    best_particle.print_map()