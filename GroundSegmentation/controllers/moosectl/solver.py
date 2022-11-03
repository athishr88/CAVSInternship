"""
Author: Athish Ram Das
Company: CAVS - MSU

Description:
    This file contains the algorithms that can be used in the 
    robot controllers
"""

from matplotlib import pyplot as plt
import numpy as np
import math


class Vector2d():
    def __init__(self, x, y):
        self.deltaX = x
        self.deltaY = y
        self.length = -1
        self.direction = [0, 0]
        self.vector2d_share()

    def vector2d_share(self):
        if type(self.deltaX) == type(list()) and type(self.deltaY) == type(list()):
            deltaX, deltaY = self.deltaX, self.deltaY
            self.deltaX = deltaY[0] - deltaX[0]
            self.deltaY = deltaY[1] - deltaX[1]
            self.length = math.sqrt(self.deltaX ** 2 + self.deltaY ** 2) * 1.0
            if self.length > 0:
                self.direction = [self.deltaX / self.length, self.deltaY / self.length]
            else:
                self.direction = None
        else:
            self.length = math.sqrt(self.deltaX ** 2 + self.deltaY ** 2) * 1.0
            if self.length > 0:
                self.direction = [self.deltaX / self.length, self.deltaY / self.length]
            else:
                self.direction = None

    def __add__(self, other):
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX += other.deltaX
        vec.deltaY += other.deltaY
        vec.vector2d_share()
        return vec

    def __sub__(self, other):
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX -= other.deltaX
        vec.deltaY -= other.deltaY
        vec.vector2d_share()
        return vec

    def __mul__(self, other):
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX *= other
        vec.deltaY *= other
        vec.vector2d_share()
        return vec

    def __truediv__(self, other):
        return self.__mul__(1.0 / other)

    def __repr__(self):
        return 'Vector deltaX:{}, deltaY:{}, length:{}, direction:{}'.format(self.deltaX, self.deltaY, self.length,
                                                                             self.direction)



class APF():
    """
    Adapted: https://github.com/ShuiXinYun/Path_Plan/tree/master/Artificial_Potential_Field
    """
    def __init__(self, start, goal, obstacles, k_att: float, k_rep: float, rr: float,
                 step_size: float, max_iters: int, goal_threshold: float, is_plot=False):

        self.start = Vector2d(start[0], start[1])
        self.current_pos = Vector2d(start[0], start[1])
        self.goal = Vector2d(goal[0], goal[1])
        self.obstacles = [Vector2d(OB[0], OB[1]) for OB in obstacles]
        self.k_att = k_att
        self.k_rep = k_rep
        self.rr = rr  
        self.step_size = step_size
        self.max_iters = max_iters
        self.iters = 0
        self.goal_threashold = goal_threshold
        self.path = list()
        self.is_path_plan_success = False
        self.is_plot = is_plot
        self.delta_t = 0.01

    def attractive(self):
        att = (self.goal - self.current_pos) * self.k_att
        return att

    def repulsion(self):
        rep = Vector2d(0, 0)
        for obstacle in self.obstacles:
            # obstacle = Vector2d(0, 0)
            t_vec = self.current_pos - obstacle
            if (t_vec.length > self.rr):
                pass
            else:
                rep += Vector2d(t_vec.direction[0], t_vec.direction[1]) * self.k_rep * (
                        1.0 / t_vec.length - 1.0 / self.rr) / (t_vec.length ** 2) 
        return rep

    def path_plan(self):
        while (self.iters < self.max_iters and (self.current_pos - self.goal).length > self.goal_threashold):
            f_vec = self.attractive() + self.repulsion()
            self.current_pos += Vector2d(f_vec.direction[0], f_vec.direction[1]) * self.step_size
            self.iters += 1
            self.path.append([self.current_pos.deltaX, self.current_pos.deltaY])
            if self.is_plot:
                plt.plot(self.current_pos.deltaX, self.current_pos.deltaY, '.b')
                plt.pause(self.delta_t)
        if (self.current_pos - self.goal).length <= self.goal_threashold:
            self.is_path_plan_success = True
    
    def get_path(self, step_size_coarse):
        self.path_plan()

        if self.is_path_plan_success:
            path_fine = self.path
            path_coarse = []

            i = int(step_size_coarse/self.step_size)
            while (i < len(path_fine)):
                path_coarse.append(path_fine[i])
                i += int(step_size_coarse/self.step_size)
            
            if path_coarse[-1] != path_fine[-1]:
                path_coarse.append(path_fine[-1])
            # print('planed path points:{}'.format(path_coarse))
            print('path plan success')

            return path_coarse

        else:
            print('path plan failed')
            return None, None


class DivNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        pass


class GroundSegmentor:
    """
    This is an object to perform ground segmentation heuristics
    to filter out point clouds that hit the ground. 

    This will only work with multi layer lidars (for this version)
    """
    def __init__(self, model):
        """model -> model instance of the MooseModel class"""
        #Parameters
        self.model = model
        self.resolution = 1 # Can be changed
        self.lidar_height = 0.8 # To be determined
        self.error = 0.2
        self.extends = 10 # In a 10+10 * 10 + 10 area

        # Tools
        self.lowest = 0
        self.highest = 0
        self.size = 19 #TODO to be removed

        self._init_seg_array()

    def _init_seg_array(self):
        size = int(self.extends / self.resolution)
        self.size = size
        x = []
        for i in range(-size+1, size):
            x.extend([i]*(2*size-1))
        y = np.arange(-size+1, size).tolist()*(2*size-1)
        dimension = ((2*size)-1)*((2*size)-1)
        self.seg_array = np.zeros(shape=(dimension, 3))
        self.seg_array[:, 0] = np.array(x)
        self.seg_array[:, 1] = np.array(y)
        self.seg_array[:, 2] = float("inf")
    
    def _mul_relative_points(self, sweep_list): # Expects sweep to be a list
        """Returns relative positions of cloud points from lidar"""
        layers = self.model.lidar.getNumberOfLayers()
        res = self.model.lidar.getHorizontalResolution()
        rel_coords_array = np.zeros((layers*res, 3),
                                    dtype=np.float32) # 3 dim array for x, y,z

        for i, point in enumerate(sweep_list):
            # TODO has to be completed
            rel_coords_array[i] = [point.x, point.y, point.z]
        
        rel_coords_array[:, 2] += self.lidar_height
        return rel_coords_array

    def _abs_points(self, sweep_list):
        """Returns the absolute position of cloud points in a numpy array"""
        coords_array = self._mul_relative_points(sweep_list)
        position_3D = self.model.gps.getValues()

        for i in range(3):
            coords_array[:, i] += position_3D[i]

        return coords_array

    def _get_ground_points_bins(self):
        dim = int(self.extends)
        bins = np.arange(-dim+1, dim, self.resolution, dtype=float).tolist()

        return bins
    
    def _find_seg_row(self, x, y):
        seg_row = x*self.size + y
        return int(seg_row)

    def _convert_to_bins(self, sweep_list):
        """Converts lidar raw data into bins"""
        rel_points = self._mul_relative_points(sweep_list)
        bins = self._get_ground_points_bins()

        raw_x = rel_points[:, 0]
        raw_y = rel_points[:, 1]

        binned_x = np.digitize(raw_x, bins)
        binned_y = np.digitize(raw_y, bins)
        rel_points[:, 0] = binned_x
        rel_points[:, 1] = binned_y

        return rel_points

    def write_to_txt(self, array):
        for i in range(array.shape[0]):
            with open("array.txt", "a") as f:
                f.write(str(array[i])+"\n")

    def find_lowest_point(self, point_cloud):
        rel_points = self._convert_to_bins(point_cloud)

        z_axis_list = rel_points[:, 2].tolist()
        
        for i, z_point in enumerate(z_axis_list):
            if abs(z_point - self.lowest) < self.error or \
                abs(z_point - self.highest) < self.error:
                # print(rel_points[i])
                x, y = rel_points[i][: 2]
                row = self._find_seg_row(x, y)
                if self.seg_array[row][2] > rel_points[i][2]:
                    self.seg_array[row][2] = rel_points[i][2]
                pass
        
        self.write_to_txt(self.seg_array)
        pass

    def divide_points(self, sweep, res):
        pass

    def ground_points(self, sweep):
        ground_coords= []
        highest_ground_point = self.find_lowest_point()
