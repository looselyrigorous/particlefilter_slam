from numpy import sign
cimport numpy as np
import numpy as np
cimport cython
from libc.math cimport floor, isnan, sin, cos,INFINITY
@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True)

cpdef np.ndarray[dtype = np.int64_t,ndim=2] plot_all_lines(float x,float y,float th,float start_angle,
                                            float end_angle,float angle_incr,
                                            np.ndarray[dtype = double,ndim=1] ranges, float max_depth,int sizeX,
                                            int sizeY,float fidelity):

    newMap = np.ndarray(shape = (sizeX,sizeY),dtype = np.int)
    newMap.fill(-1)
    cdef float x0 = x
    cdef float y0 = y
    cdef float x1
    cdef float y1
    start_angle += th
    cdef int angles = len(ranges)
    angle_incr = (end_angle - start_angle)/angles
    cdef int count
    cdef float distance
    cdef float curr_angle
    cdef bint Wall
    cdef int xS,yS,xE,yE
    ##### First we make the empty space

    for count in range(0,angles):
        curr_angle = start_angle + (angle_incr*count)
        distance = ranges[count]
        Wall = True
        if distance == INFINITY or isnan(distance):
            distance = max_depth
            Wall = False
        x1 = x0 + distance*cos(curr_angle)
        y1 = y0 + distance*sin(curr_angle)

        xS,yS = coord_to_grid_coord(x0,y0,fidelity,sizeX,sizeY)
        xE,yE = coord_to_grid_coord(x1,y1,fidelity,sizeX,sizeY)
        newMap = zero_tiler(newMap,xS,yS,xE,yE)

    # Then we add all the occupied space

    for count in range(0,angles):
        distance = ranges[count]
        if distance == INFINITY or isnan(distance):
            continue
        curr_angle = start_angle + (angle_incr*count)
        x1 = x0 + distance*cos(curr_angle)
        y1 = y0 + distance*sin(curr_angle)
        xE , yE = coord_to_grid_coord(x1, y1, fidelity, sizeX, sizeY)
        if xE>=0 and xE<sizeX and yE>=0 and yE <sizeY:
            newMap[xE,yE] = 1

    return newMap


cdef np.ndarray[dtype = np.int64_t,ndim = 2] zero_tiler(np.ndarray[dtype = np.int64_t,ndim = 2] newMap, int x0, int y0, int x1, int y1):
        PsizeX = newMap.shape[0]
        PsizeY = newMap.shape[1]

        if x0 == x1:
            if y0 == y1:
                return newMap
            else:
                newMap = plot_lineY(newMap,y0,y1,x1)

        elif y0==y1:
            newMap = plot_lineX(newMap,x0,x1,y1)

        elif (abs(x0-x1)>abs(y0-y1)):
            newMap = plot_lineX_high(newMap,x0,x1,y0,y1)

        else:
            newMap = plot_lineY_high(newMap,x0,x1,y0,y1)

        return newMap



cdef np.ndarray[dtype = np.int64_t,ndim=2] plot_lineX(np.ndarray[dtype = np.int64_t,ndim = 2] newMap,int x0,int x1,int y):
    cdef int PsizeX = newMap.shape[0]
    cdef int step = sign(x1-x0)
    cdef int x

    if step==-1: ## This is for cython optimazation purposes
        for x in range(x0,x1,-1):
            if x<0 or x>=PsizeX:
                return newMap
            newMap[x,y] = 0

    else:
        for x in range(x0,x1,1):
            if x<0 or x>=PsizeX:
                return newMap
            newMap[x,y] = 0

    return newMap

cdef np.ndarray[dtype = np.int64_t,ndim = 2] plot_lineY(np.ndarray[dtype = np.int64_t,ndim = 2] newMap,int x,int y0,int y1):
    cdef int PsizeY = newMap.shape[1]
    cdef int step = sign(y1-y0)
    cdef int y
    if step==-1: ## This is for cython optimazation purposes
        for y in range(y0,y1,-1):
            if y<0 or y>=PsizeY:
                return newMap
            newMap[x,y] = 0

    else:
        for y in range(y0,y1,1):
            if y<0 or y>=PsizeY:
                return newMap
            newMap[x,y] = 0

    return newMap

cdef np.ndarray[dtype = np.int64_t,ndim = 2] plot_lineX_high(np.ndarray[dtype = np.int64_t,ndim = 2] newMap,int x0,int x1,int y0,int y1):
    cdef int PsizeX = newMap.shape[0]
    cdef int PsizeY = newMap.shape[1]

    cdef int step = sign(x1-x0)
    cdef float error = abs(float(y0 - y1) / float(x0 - x1))
    cdef float Yerror = 0
    cdef int Ystep = sign(y1-y0)
    cdef int y = y0
    cdef int x

    if step == -1:
        for x in range(x0,x1,-1):
            if x<0 or x>=PsizeX:
                return newMap
            newMap[x,y] = 0
            Yerror +=error
            if Yerror>=1:
                y += Ystep
                if y<0 or y>=PsizeY:
                    return newMap
                newMap[x,y] = 0
                Yerror -= 1
    else:
        for x in range(x0,x1):
            if x<0 or x>=PsizeX:
                return newMap
            newMap[x,y] = 0
            Yerror +=error
            if Yerror>=1:
                y += Ystep
                if y<0 or y>=PsizeY:
                    return newMap
                newMap[x,y] = 0
                Yerror -= 1

    return newMap

cdef np.ndarray[dtype = np.int64_t,ndim = 2] plot_lineY_high(np.ndarray[dtype = np.int64_t,ndim = 2] newMap,int x0,int x1,int y0,int y1):
    cdef int PsizeX = newMap.shape[0]
    cdef int PsizeY = newMap.shape[1]
    cdef float error = abs(float(x0-x1)/float(y0-y1))
    cdef float Xerror = 0
    cdef int step = sign(y1-y0)
    cdef int Xstep = sign(x1-x0)
    cdef int x = x0
    cdef int y
    if step==-1:
        for y in range(y0,y1,-1):
            if y<0 or y>=PsizeY:
                return newMap
            newMap[x,y] = 0
            Xerror +=error
            if Xerror>=1:
                x += Xstep
                if x<0 or x>=PsizeX:
                    return newMap
                newMap[x,y] = 0
                Xerror -= 1
    else:
        for y in range(y0,y1):
            if y<0 or y>=PsizeY:
                return newMap
            newMap[x,y] = 0
            Xerror +=error
            if Xerror>=1:
                x += Xstep
                if x<0 or x>=PsizeX:
                    return newMap
                newMap[x,y] = 0
                Xerror -= 1

    return newMap


cpdef coord_to_grid_coord(float x, float y, float fidelity, int sizeX, int sizeY):

    x *= 2 / fidelity
    y *= 2 / fidelity

    x += (sizeX - 1) / 2
    y += (sizeY - 1) / 2

    return int(floor(x)), int(floor(y))



cpdef void print_map(np.ndarray[dtype = np.int64_t,ndim = 2] Map):
    PsizeX = Map.shape[0]
    PsizeY = Map.shape[1]
    cdef int i
    cdef int j
    cdef int occ
    for i in range(0, PsizeX):
        stringer = ''
        for j in range(0, PsizeY):
            occ = Map[i,j]
            if occ == 1:
                stringer += 'x'
            elif occ == 0:
                stringer += 'o'
            else:
                stringer += ' '
        print(stringer)
