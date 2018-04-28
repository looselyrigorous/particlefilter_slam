import numpy as np
cimport numpy as np
from libc.math cimport isnan,INFINITY


@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True)


###This patches scan nans

cpdef np.ndarray[dtype = double, ndim=1] trump_walling(np.ndarray[dtype = double, ndim=1] ranges,int distance):
    cdef int angles = ranges.shape[0]
    new_ranges = ranges.copy()

    cdef int i,j
    cdef double value
    cdef double sum,tmp_value
    cdef int search_pos,hits
    for i in range(0,angles):
        value = ranges[i]
        if value == INFINITY or isnan(value):
            print("hey")
            sum = 0
            hits = 0
            for j in range(0,distance):
                search_pos = i+j
                if search_pos >= angles:
                    break
                tmp_value = ranges[distance]
                if tmp_value!= INFINITY and not isnan(tmp_value):
                    sum += tmp_value
                    hits +=1
            for j in range(0,distance):
                search_pos = i-j
                if search_pos <0:
                    break
                tmp_value = ranges[distance]
                if tmp_value != INFINITY and not isnan(tmp_value):
                    sum += tmp_value
                    hits += 1

            if sum != 0:
                new_ranges[i] = sum/hits
                print('hell yeah hit it :)' + str(hits))
    return new_ranges