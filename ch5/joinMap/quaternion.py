#!/usr/bin/python3
import numpy as np

class Quaternion:
    def __init__(self, w, x, y, z):
        '''
        @param: w, x, y, z
        '''
        self.w = w
        self.x = x
        self.y = y
        self.z = z
        self.vector = np.array([w, x, y, z])

    def __str__(self):
        imaginary = [' ', 'i ', 'j ', 'k]']
        result = '['
        for i in range(4):
            result = result + str(self.vector[i]) + imaginary[i]
        return result

    def __add__(self, quater):
        q = self.vector + quater.vector
        return Quaternion(q[0], q[1], q[2], q[3])

    def transformToRotationMatrix(self):
        q = self.vector.copy()
        # w, x, y, z = q[0], q[1], q[2], q[3]
        # the rotation matrix below was transposed
        r = np.array([
            [1 - 2*q[2]*q[2] - 2*q[3]*q[3], 2*q[1]*q[2] + 2*q[0]*q[3], 2*q[1]*q[3]-2*q[0]*q[2],   0],
            [2*q[1]*q[2] - 2*q[0]*q[3], 1 - 2*q[1]*q[1] - 2*q[3]*q[3], 2*q[2]*q[3] + 2*q[0]*q[1], 0],
            [2*q[1]*q[3] + 2*q[0]*q[2], 2*q[2]*q[3] - 2*q[0]*q[1], 1 - 2*q[1]*q[1] - 2*q[2]*q[2], 0],
            [0,     0,      0,      1]
        ])
       
        return r.transpose()

def translation(position):
    '''
    @param: position: a 1x3 numpy array, indicates the location of a point (x,y,z)
    '''
    x, y, z = position[0], position[1], position[2]

    T = np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])
    return T