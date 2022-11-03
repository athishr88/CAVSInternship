"""
Author: Athish Ram Das

Description:
    This file contains the primary application class,
    which instantiates the primary objects, and creates
    the appropriate assignments.
"""

from models import MooseModel


class Main:
    """
    Description:
        This app class is the primary object that
        instiates the system.
    """
    def __init__(self):
        self.model = MooseModel()
        pass

    

if __name__ == "__main__":
    moose = Main()

    #__________AP____________________
    # start, goal = (14.2, -5.6), (-2.8, 13.2)
    # obs = [[14.9, 3.77], [11.6, 3.96], [7.66, 4.21], [3.17, 4.15]]
    # path = moose_model.compute_path(start, goal, obs)
    # moose_model.set_AP_path(path)
    # moose_model.set_AP_mode(1)

    #________________Lidar____________
    # moose.model.all_points()
    # print(moose.model.lidar.isPointCloudEnabled())
    # sweep = moose.model.lidar.getPointCloud()
    # print(sweep[0].x)
    # moose.model.all_points(sweep)

    # ________Misc____________________
    moose.model.robot.step(200)
    sweep = moose.model.lidar.getPointCloud()
    moose.model.segmentor.find_lowest_point(sweep)

    #_________RUN Loop________________
    # moose.model.run_loop()


# #    ___________________________Testing area__________________________________
# from models import MooseModel

# model = MooseModel()
# while model.robot.step(model.timestep) != -1:
#     sweep = model.lidar.getPointCloud()
#     print(sweep[0].z)