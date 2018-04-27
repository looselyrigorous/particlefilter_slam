from numpy import sign
import Slam as Slam
import numpy as np
cimport cython
cimport numpy as np



cdef int Psize = Slam.mapSize

@cython.boundscheck(False)
@cython.wraparound(False)


cpdef np.ndarray[int,ndim=2] plotLineX(np.ndarray[int,ndim = 2] newMap,int x0,int x1,int y):
    # Cython speed-up implementation

    cdef int x
    cdef int step = sign(x1-x0)

    if step == -1:
        x0,x1 = x1,x0

    cdef int xS = x0

    for x in range(x0,x1):
        if x>=0 and x<Psize:
            xS = x
            break
    #################################

    for x in range(xS,x1):
        if x<0 or x>=Psize:
            return newMap
        newMap[x,y] = 0

    return newMap

cpdef np.ndarray[int,ndim = 2] plotLineY(np.ndarray[int,ndim = 2] newMap,int x,int y0,int y1):
    # Cython speed-up implementation

    cdef int y
    cdef int step = sign(y1-y0)

    if step == -1:
        y0,y1 = y1,y0

    cdef int yS = y0

    for y in range(y0,y1):
        if y>=0 and y<Psize:
            yS = y
            break
    ################################

    for y in range(yS,y1):
        if y<0 or y>=Psize:
            return newMap
        newMap[x,y] = 0

    return newMap

cpdef np.ndarray[int,ndim = 2] plotLineXHigh(np.ndarray[int,ndim = 2] newMap,int x0,int x1,int y0,int y1):

    cdef int step = sign(x1-x0)
    cdef float error = abs(float(y0 - y1) / float(x0 - x1))

    #Cython speed-up implementation

    cdef float Yerror = 0
    if step == -1:
        x0,x1 = x1,x0
        y0,y1 = y1,y0
    cdef int Ystep = sign(y1-y0)
    cdef int y = y0
    cdef int x
    cdef bint Xflag = False
    cdef bint Yflag = False
    cdef int xS = x0

    for x in range(x0,x1):
        Yerror += error

        if Yerror >=1:
            y+= Ystep

        if x>=0 and x<Psize:
            Xflag == True

        if y>=0 and y<Psize:
            Yflag = True

        if Xflag and Yflag:
            xS = x

    ###########################

    for x in range(xS,x1):
        if x<0 or x>=Psize:
            return newMap
        newMap[x,y] = 0
        Yerror +=error
        if Yerror>=1:
            y += Ystep
            if y<0 or y>=Psize:
                return newMap
            newMap[x,y] = 0
            Yerror -= 1
    return newMap


cpdef np.ndarray[int,ndim = 2] plotLineYHigh(np.ndarray[int,ndim = 2] newMap,int x0,int x1,int y0,int y1):
    cdef float error = abs(float(x0-x1)/float(y0-y1))
    cdef float Xerror = 0
    cdef int step = sign(y1-y0)

    # Cython Speed-up Implementation
    if step == -1:
        y0,y1 = y1,y0
        x0,x1 = x1,x0

    cdef int Xstep = sign(y1-y0)
    cdef int x = x0
    cdef int y
    cdef bint Xflag = False
    cdef bint Yflag = False
    cdef int xS = x0
    cdef int yS = y0

    for y in range(y0,x1):
        Xerror += error

        if Xerror >=1:
            x+= Xstep

        if x>=0 and x<Psize:
            Xflag == True

        if y>=0 and y<Psize:
            Yflag = True

        if Xflag and Yflag:
            yS = y

    #################################


    for y in range(yS,y1):
        if y<0 or y>=Psize:
            return newMap
        newMap[x,y] = 0
        Xerror += error
        if Xerror>=1:
            x+=Xstep
            if x<0 or x>=Psize:
                return newMap
            newMap[x,y] = 0
            Xerror -= 1

    return newMap