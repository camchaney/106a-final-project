#!/usr/bin/env python3

class Voxelizer(object):

    def __init__(self, resolution=.001):
        """
        Voxelizes positions in 3d space for efficient set lookup

        resolution: float
            The size of a voxel
        """
        self.resolution = resolution

    def convert(self, x_y_z_tuple):
        return tuple([int(value//self.resolution) for value in x_y_z_tuple])