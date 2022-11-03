"""
Author: Athish Ram Das
Company: CAVS - MSU

Description:
    The Commander class is responsible for executing controller
    commands on the robot
"""

import numpy as np
import math

class Commander:
    """This is an object that offers several control functions 
    of the robot"""
    def __init__(self, model):
        """All default hyperparameters of the robot controller 
        is defined here"""

        # hyperparameters      
        self.speeds = [0.0, 0.0]
        self.MAX_SPEED = 7.0

        # autopilot
        self.AUTOPILOT = False
        self.current_target_index = None
        self.TARGET_POINTS_SIZE = 13
        self.DISTANCE_TOLERANCE = 0.5
        self.TURN_COEFFICIENT = 4.0
        
        # external instances
        self.model = model

    def _modulus_double(self, a, m):
        div = int(a/m)
        r = a - div * m
        if r < 0.0:
            r += m
        return r

    def _minus(self, v1, v2):
        v1 = np.array(v1)
        v2 = np.array(v2)
        return v1-v2

    def robot_set_speed(self,left, right):
        """Sets speed to all the 8 wheels of the moose robot"""
        i = 0
        for i in range(4):
            self.model.motors[i + 0].setPosition(float("inf"))
            self.model.motors[i + 4].setPosition(float("inf"))
            self.model.motors[i + 0].setVelocity(left)
            self.model.motors[i + 4].setVelocity(right)

    def follow_ap_path(self,path):
        """Provide a list of paths and the motor speeds of the robots 
        are set correspondingly"""

        # compute the 2D position of the robot and its orientation
        speeds = [0.0, 0.0]
        position_3d = self.model.gps.getValues()
        north_3d = self.model.compass.getValues()
        position = [position_3d[0], position_3d[1]]
        
        #compute the direction and the distance to the target
        direction = self._minus(path[self.current_target_index], position)
        distance = np.linalg.norm(direction)
        direction /= distance
        
        # compute the error angle
        robot_angle = math.atan2(north_3d[0], north_3d[1])
        target_angle = math.atan2(direction[1], direction[0])
        beta = self._modulus_double(target_angle - robot_angle, 2.0 * math.pi) \
                                    - math.pi
        
        
        # move singularity
        if beta > 0:
            beta = math.pi - beta
        else:
            beta = -beta - math.pi
        
        # a target position has been reached
        if distance < self.DISTANCE_TOLERANCE:
            index_char = "th"
            if self.current_target_index == 0:
                index_char = "st"
            elif self.current_target_index == 1:
                index_char = "nd"
            elif self.current_target_index == 2:
                index_char = "rd"
            print(f"{self.current_target_index + 1}{index_char} target reached. \
                        {self.current_target_index}\n")
            self.current_target_index += 1
            self.current_target_index %= self.TARGET_POINTS_SIZE
        else:
            speeds[0] = self.MAX_SPEED - math.pi + self.TURN_COEFFICIENT * beta
            speeds[1] = self.MAX_SPEED - math.pi - self.TURN_COEFFICIENT * beta
            
        
        self.robot_set_speed(speeds[0], speeds[1])
        if self.current_target_index > len(path)-1:
            self.robot_set_speed(0, 0)
            self.model.AUTOPILOT = False
        pass
