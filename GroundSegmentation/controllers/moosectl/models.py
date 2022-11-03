"""
Author: Athish Ram Das

Description:
    The MooseModel class is responsible for communicating
    with the backend systems.
"""

from controller import Robot, Camera, Lidar, GPS, Compass, Motor, Keyboard
from solver import APF, GroundSegmentor
from robocommands import Commander
import math
import numpy as np


class MooseModel:
    """
    This is a contoller object that can initiates the 
    basic controller for the moose. Methods of the 
    class can add functionalities
    """

    def __init__(self):

        # creating instances
        self.robot = Robot()
        self.front_cam = Camera("camera")
        self.lidar = Lidar("Velodyne HDL-32E")
        self.gps = GPS("gps")
        self.compass = Compass("compass")
        
        # Statics
        # self.timestep = int(self.robot.getBasicTimeStep())
        self.timestep = 64
        self.ap_path = None
        self.AUTOPILOT = False
        self.GROUNDSEGMENTATION = True

        # Enabling devices
        self._init_lidar()
        self._init_gps()
        self._init_motors()
        self._init_compass()
        self._init_camera()


        # External libraries
        self.commander = Commander(self)
        self.segmentor = GroundSegmentor(self)

    # _____________Device inits_________________
    def _init_camera(self):
        self.front_cam.enable(self.timestep)

    def _init_motors(self): # Set parameters here
        self.motors = []
        motor_names = ["left motor 1",  "left motor 2",  "left motor 3",
                       "left motor 4", "right motor 1", "right motor 2",
                       "right motor 3", "right motor 4"]
        
        for m in range(8):
            self.motors.append(self.robot.getDevice(motor_names[m]))
    
    def _init_compass(self): # Set parameters here
        self.compass.enable(self.timestep)

    def _init_gps(self): # Set parameters here
        self.gps.enable(self.timestep)


    def _init_lidar(self): # Set parameters here
        # Lidar
        print("Initializing lidar")
        self.lidar.enable(self.timestep)
        self.lidar.setFrequency(5)
        self.lidar.enablePointCloud()

    # ________________Navigation___________________

    def set_AP_path(self, path): # Set autopilot path 
        self.commander.current_target_index = 0
        self.ap_path = path
        # Set autopilot mode on to start navigation
    
    def set_AP_mode(self, mode): # Set autopilot mode on or off
        self.AUTOPILOT = False if mode == 0 else True

    def _follow_path(self):
        self.commander.follow_ap_path(self.ap_path)

    def compute_path(self, start, goal, obstacles, k_att=1, k_rep=500.0,
                     rr=10, step_size=0.2, max_iters=500,
                     goal_threshold = 0.2, is_plot=False, step_size_=3):
        """Computes path and returns coarse paths"""
        
        planner = APF(start, goal, obstacles, k_att, k_rep,
                     rr, step_size, max_iters,
                     goal_threshold, is_plot)
        
        return planner.get_path(step_size_)

    # __________________Algos_____________________     
    # TODO to be deleted 
    def get_ground(self):
        sweep = self.lidar.getPointCloud()
        x = self.segmentor._convert_to_bins(sweep)
        print(x[-5][2])

    def run_loop(self):
        """Runs an infinity while loop"""
        while self.robot.step(self.timestep) != -1:
            if self.AUTOPILOT:
                self._follow_path()
            if self.GROUNDSEGMENTATION:
                self.get_ground()
        pass

