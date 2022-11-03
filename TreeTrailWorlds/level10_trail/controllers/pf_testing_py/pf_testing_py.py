from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())
    
current_target_index = 0        
while robot.step(timestep) != -1:
    # code here
    
    pass
    