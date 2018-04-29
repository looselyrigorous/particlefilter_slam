cimport numpy as np
import numpy as np
cimport cython

from libc.math cimport sqrt,ceil


cpdef float map_radius_error_calc(np.ndarray[dtype = np.int64_t, ndim=2] merger_map,
                            np.ndarray[dtype = np.int64_t, ndim=2] grid,
                            np.ndarray[dtype = double, ndim=2] prop_map,
                            np.ndarray[dtype = np.int64_t, ndim=2] tick_map,):

    cdef int PsizeX = grid.shape[0]
    cdef int PsizeY = grid.shape[1]
    cdef int x,y
    cdef error = 0
    for x in range(0,PsizeX):
        for y in range(0,PsizeY):
            if merger_map[x,y] != -1 and grid[x,y]!=-1:
                error += abs_tile_radius_error(merger_map[x,y],grid,x,y)

    return 1/(error+1)



cdef double abs_tile_radius_error(int tile,np.ndarray[dtype = np.int64_t, ndim=2] grid,int x,int y):

    cdef int PsizeX = grid.shape[0]
    cdef int PsizeY = grid.shape[1]

    cdef int max_radius = int(ceil(sqrt(PsizeX*PsizeX + PsizeY*PsizeY)))
    cdef int i

    for i in range(0,max_radius):
        if calc_error_radius(grid,x,y,i,tile):
            return i
    return max_radius

cdef calc_error_radius(np.ndarray[dtype = np.int64_t, ndim=2] grid, int x0, int y0, int radius, int tile):
    cdef int x = 0
    cdef int y = 0
    cdef int dx = 1
    cdef int dy = 1
    cdef int err = dx - (radius * 2)

    cdef int count = 0

    while x >= y:
        count += checkSimilarity(tile, x0 + x, y0 + y, grid)
        count += checkSimilarity(tile, x0 + x, y0 - y, grid)
        count += checkSimilarity(tile, x0 - x, y0 + y, grid)
        count += checkSimilarity(tile, x0 - x, y0 - y, grid)
        count += checkSimilarity(tile, x0 + y, y0 + x, grid)
        count += checkSimilarity(tile, x0 + y, y0 - x, grid)
        count += checkSimilarity(tile, x0 - y, y0 + x, grid)
        count += checkSimilarity(tile, x0 - y, y0 - x, grid)

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


cdef int checkSimilarity(int tile, int x, int y, np.ndarray[dtype = np.int64_t, ndim=2] grid):
    cdef int PsizeX = grid.shape[0]
    cdef int PsizeY = grid.shape[1]

    if x < PsizeX and x >= 0 and y < PsizeY and y >= 0:
        if tile == grid[x,y]:
            return 1
        else:
            return 0
    return 0