"""
Author: Athish Ram Das
Company: Center for Advanced Vehicle Systems, MSU

Description:
    WebotsWorldCreator can create a webots world from scratch upon
    instructions about the creation of an object and the location of
    its placement

    This file contains the primary application class,
    which instantiates the primary objects, and creates
    the appropriate assignments.
"""
from email.policy import default
import math
import time


class ModifyWorld():
    """
    Description
        This ModifyWorld class is the object that can provide services 
        to continually modify the webots world 
    """
    def __init__(self, world):
        self.world = world
        pass

    def modify_uneven_terrain(self,x, y, height, x_dim=49, y_dim=49):
        """
        Params
        """
        # Look for "height" in the file and grab all the height values for replacement
        with open(self.world.world_file, "r") as f:
            all_lines = f.readlines()
        all_heights = []
        flag = 0
        for i, line in enumerate(all_lines):
            if "height" in line:
                flag = 1
            elif flag == 1 and "]" not in line:
                all_heights.extend(line.strip().split(","))
            if flag == 1 and "]" in line:
                flag = 0
                break

        # Converting coordinate system from edge(default of uneven terrain) to center
        height_list_index = math.ceil(x_dim*y_dim/2) + (y*(x_dim)+x)
        all_heights[height_list_index] = str(height)
        heights_string = ", ".join(all_heights)
        heights_string += "\n"
        
        all_lines[i-1] = heights_string
        with open(self.world.world_file, "r+") as f:
            f.writelines(all_lines)

class WorldGenerator():
    """
    Description:
        This WorldGenerator class is the primary object that
        initiates the system.
    """

    """
    General instruction for arguments usage in node import: 

    formats/default values:
    translation = "0 0 0"
    rotation = "0 0 1 0"
    floor_size = "1 1"
    floor_tile_size = "0.5 0.5"
    floor_appearance = "checked_box"
    wall_height = "0.1"

    name usage: name="\"<name>\""
    """

    def __init__(self, filename):

        # Initialize a world and add textured background and light
        self.world_file = filename
        with open(self.world_file, 'w') as f:
            f.write('#VRML_SIM R2022a utf8\nWorldInfo {\n}\nViewpoint {\n  orientation -0.5773 0.5773 0.5773 2.0944\n  position 0 0 10\n}\nTexturedBackground {\n}\nTexturedBackgroundLight {\n}\n')

        pass

    def scripter(self, func_args, file):
        # Function to add any node to the world file
        with open(self.world_file, 'a') as world_file:
            with open(file, 'r') as form_file:
                world_file.write("\n")
                for i , line in enumerate(form_file):
                    if "%" in line:
                        if func_args[i] != default:
                            world_file.write(line %func_args[i])
                        else:
                            pass
                    else:
                        world_file.write(line)

    def add_rectangle_arena(self, translation=default, rotation=default, 
                            name=default, floor_size=default, 
                            floor_tile_size=default,
                            floor_appearance=default, wall_height= default):
        func_args = [None, translation, rotation, name, floor_size,
                     floor_tile_size, floor_appearance, None, wall_height, None]

        form_file = "mapGenerator/Nodes/rectangular_arena.txt"
        
        self.scripter(func_args, form_file)

    def add_cypress_tree(self, translation=default, rotation=default, name=default):
        func_args = [None, translation, rotation, name, None]
        form_file = "mapGenerator/Nodes/cypress_tree.txt"

        self.scripter(func_args, form_file)
    
    def add_straight_road_segment(self, translation="0 0 .01", rotation=default, name=default,
                                  id=default, width=default, numberOfLanes=default,
                                  numberOfForwardLanes=default, speedLimit=default,
                                  length=default):
        func_args = [None, translation, rotation, name, id, width, numberOfLanes,
                    numberOfForwardLanes, speedLimit,length, None, None]
        form_file = "mapGenerator/Nodes/straight_road_segment.txt"
        self.scripter(func_args, form_file)

    def add_uneven_terrain(self, translation="0 0"):

        # Uneven terrain is translated assuming that the xdim and ydim are 50 each 
        processed_translation = " ".join([str(float(i)-26) for i in translation.split(" ")]) 
        with open(self.world_file, 'a') as world_file:
            with open("mapGenerator/Nodes/uneven_terrain.txt", 'r') as form_file:
                world_file.write("\n")
                for line in form_file:
                    if "%" in line:
                        world_file.write(line %processed_translation)
                    else:
                        world_file.write(line)
    def add_moose_robot(self):
        # All values changed to accomodate moose into default uneven surface floor
        form_file = "mapGenerator/Nodes/moose.txt"
        func_args = [None]
        self.scripter(func_args, form_file)



if __name__ == "__main__":
    st = time.time()
    world = WorldGenerator('worlds/world.wbt')

    # world.add_rectangle_arena(floor_size="100 100", floor_appearance="Sand",
    #                         name="\"rectangle arena\"")
    # for i in range(10):
    #     world.add_cypress_tree(name="\"First tree%d\"" %i, translation="%d 5 0" %(i*5))
    #     world.add_cypress_tree(name="\"First tree%d\"" %(i+10), translation="%d -5 0" %(i*5))
    # world.add_straight_road_segment(length="50")
    world.add_uneven_terrain()
    world.add_moose_robot()
    world.add_cypress_tree("5 5 3.5")
    modifier = ModifyWorld(world)
    modifier.modify_uneven_terrain(10, 2, 4.5)
    modifier.modify_uneven_terrain(9, 1, 4.2)
    modifier.modify_uneven_terrain(8, 0, 4)
    et = time.time()
    print(et-st)

