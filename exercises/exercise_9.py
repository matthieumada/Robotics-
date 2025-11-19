
import numpy as np
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt
import random

from ompl import base as ob
from ompl import geometric as og

from robot import *

import numpy as np
import roboticstoolbox as rtb
from scipy.optimize import minimize
from spatialmath import SE3, SO3
import mujoco

# use for slow motion 
def linear_interpolation(start_q,end_q,steps,robot, gripper):
    q0 = start_q
    qf = end_q
    t0 = 0
    tf = 1
    T = tf-t0
    # Calculate of pisition with linear interpolation
    for t in np.linspace(t0, tf, steps):
        q_t = q0 + (t-t0)* (qf-q0) /T 
        robot.queue.append((q_t,gripper )) # qpose and gripper value
    

# use for big and fast  motion 
def parabolic_interpolation(start_q, end_q, steps,robot, gripper):
    q0 = start_q
    qf = end_q
    t0 = 0
    tf = 1
    td = tf - t0 # duration of travel
    tb = tf * 0.3 # blend time

    # Calculate required acceleration for the blend
    ddqb = (qf-q0) / (tb * (td -tb)) # constant acceleration during blend
    print(" Consntant acceleration during blend: ", ddqb)
    for t in np.linspace(t0, tf, steps):
        # acceleration phase
        if (t0<=t) and (t<t0 + tb):
            q_t = q0 + 0.5 * ddqb *(t-t0)**2

        elif ((t0 + tb)<=t) and (t< tf - tb):
            q_t = q0 + ddqb*tb*(t-t0-tb/2)


        elif tf-tb <=t and t<tf:
            q_t = qf - 0.5*ddqb*(tf - t)**2
        robot.queue.append((q_t,gripper))


# define the speed if too fast the box will be thrwon
def random_box_pos(m, d):
    selected_pos = []
    for i in range(1, 7):
        STOP = False
        while not STOP:
            rand_x = random.uniform(0.45+0.25,0.75-0.25)
            rand_y = random.uniform(-0.55+0.25,0.55-0.25)
            if len(selected_pos) == 0:
                STOP = True
                break

            SHOULD_STOP = True   
            for pos in selected_pos:
                dist = np.linalg.norm(np.array([rand_x, rand_y]) - np.array(pos))
                if dist <= 0.15:
                    SHOULD_STOP = False
                    break
            
            if SHOULD_STOP:
                STOP = True

        z = 0.025
        d.joint(f'box{i}').qpos [0:3]= [rand_x, rand_y, z+0.01]
        print(f'box{i}: ', rand_x, rand_y)
        selected_pos.append([rand_x, rand_y])

    mujoco.mj_step(m, d)

    
def move_to_target(model, data, robot, start, name, gripper):
    # Frame 10 cm above the target 
    box_frame = get_mjobj_frame(model=model, data=data, obj_name=name) * sm.SE3.Rx(-np.pi)*sm.SE3.Tz(-0.1)          
    goal_q = robot.robot_ur5.ik_LM(Tep=box_frame, q0=start)[0]
    parabolic_interpolation(start_q=start, end_q=goal_q, steps=300, robot=robot, gripper=gripper)
    print("Robot arm above to box -->  gO slowly and catch it")
    return goal_q

def move_close(model, data, robot, start, name, gripper):
    box_frame = get_mjobj_frame(model=model, data=data, obj_name=name) * sm.SE3.Rx(-np.pi)        
    goal_q = robot.robot_ur5.ik_LM(Tep=box_frame, q0=start)[0]
    linear_interpolation(start_q=start, end_q=goal_q, steps=300, robot=robot, gripper=gripper)
    print("Robot arm close to box -->  catch it")
    return goal_q

def pyramid(model, data, robot, start, name, gripper):
    benchmark = "drop_point_box2" 
    
    # box 1
    if int(name[-1]) == 1: 
        print("Box base 1")
        start = move_to_target(model, data, robot, start, benchmark, gripper)
        # move closely 
        box_frame = get_mjobj_frame(model=model, data=data, obj_name=benchmark) * sm.SE3.Rx(-np.pi) * sm.SE3.Ty(0.03)*sm.SE3.Tz(0.25) 
        goal_q = robot.robot_ur5.ik_LM(Tep=box_frame, q0=start)[0]
        linear_interpolation(start_q=start, end_q=goal_q, steps=300, robot=robot, gripper=gripper)
        gripper = 0 
        robot.set_gripper(value = gripper)
        start = move_to_target(model,data, robot, start, benchmark, gripper)
        return start
    
    # box 2
    if int(name[-1]) == 2: 
        print("Box base 2")
        start = move_to_target(model, data, robot, start, benchmark, gripper)
        # move closely 
        box_frame = get_mjobj_frame(model=model, data=data, obj_name=benchmark) * sm.SE3.Rx(-np.pi) * sm.SE3.Tz(0.25) 
        goal_q = robot.robot_ur5.ik_LM(Tep=box_frame, q0=start)[0]
        linear_interpolation(start_q=start, end_q=goal_q, steps=300, robot=robot, gripper=gripper)
        gripper = 0 
        robot.set_gripper(value = gripper)
        start = move_to_target(model,data, robot, start, benchmark, gripper)
        return start
    
    # box 3
    if int(name[-1]) == 3: 
        print("Box base 3")
        start = move_to_target(model, data, robot, start, benchmark, gripper)
        # move closely 
        box_frame = get_mjobj_frame(model=model, data=data, obj_name=benchmark) * sm.SE3.Rx(-np.pi) * sm.SE3.Ty(-0.03) *sm.SE3.Tz(0.25) 
        goal_q = robot.robot_ur5.ik_LM(Tep=box_frame, q0=start)[0]
        linear_interpolation(start_q=start, end_q=goal_q, steps=300, robot=robot, gripper=gripper)
        gripper = 0 
        robot.set_gripper(value = gripper)
        start = move_to_target(model,data, robot, start, benchmark, gripper)
        return start
    
    # box 4 
    if int(name[-1]) == 4: 
        print("Box shelve 2")
        start = move_to_target(model, data, robot, start, benchmark, gripper)
        # move closely 
        box_frame = get_mjobj_frame(model=model, data=data, obj_name=benchmark) * sm.SE3.Rx(-np.pi) * sm.SE3.Ty(-0.015)* sm.SE3.Tz(0.19) 
        goal_q = robot.robot_ur5.ik_LM(Tep=box_frame, q0=start)[0]
        linear_interpolation(start_q=start, end_q=goal_q, steps=300, robot=robot, gripper=gripper)
        gripper = 0 
        robot.set_gripper(value = gripper)
        start = move_to_target(model,data, robot, start, benchmark, gripper)
        return start 
    
    # box 5 
    elif int(name[-1]) == 5: 
        print("Box shelve 2 ")
        start = move_to_target(model, data, robot, start, benchmark, gripper)
        # move closely 
        box_frame = get_mjobj_frame(model=model, data=data, obj_name=benchmark) * sm.SE3.Rx(-np.pi) * sm.SE3.Ty(0.015)* sm.SE3.Tz(0.19) 
        goal_q = robot.robot_ur5.ik_LM(Tep=box_frame, q0=start)[0]
        linear_interpolation(start_q=start, end_q=goal_q, steps=300, robot=robot, gripper=gripper)
        gripper = 0 
        robot.set_gripper(value = gripper)
        start = move_to_target(model,data, robot, start, benchmark, gripper)
        return start 
    
    # box 6 (top box)
    else:
        print("Top box congratulations")
        start = move_to_target(model, data, robot, start, benchmark, gripper)
        # move closely 
        box_frame = get_mjobj_frame(model=model, data=data, obj_name=benchmark) * sm.SE3.Rx(-np.pi) * sm.SE3.Tz(-0.13) 
        goal_q = robot.robot_ur5.ik_LM(Tep=box_frame, q0=start)[0]
        linear_interpolation(start_q=start, end_q=goal_q, steps=300, robot=robot, gripper=gripper)
        gripper = 0 
        robot.set_gripper(value = gripper)
        start = move_to_target(model,data, robot, start, benchmark, gripper)
        return start 
    
def program(d, m):
    
    # Randomize box positions
    random_box_pos(m, d)

    name = []
    for j in range(1,4):
        name.append("box"+str(j))

    # Define our robot object
    robot = UR5robot(data=d, model=m)

    # EXERCISE: Create/implement a program that stacks the blocks on the dropzone
    # - Localize the blocks and define the grasping frames
    # - Path plan the pick and place trajectory for each block for stacking
   
    # Example: In the new scene you can get the box frames with "get_mjobj_frame" with each box named box1, box2, ....
    # get the box frame
    start = robot.get_current_q()
    for id in name:
        gripper = 0
        # move 10 cm above the box 
        start = move_to_target(m,d,robot, start, id, gripper)

        # move closely to the box 
        start = move_close(m,d,robot, start, id, gripper)
        print(id +" catched" )
        gripper =  255
        robot.set_gripper(value=gripper)
        # move 10 cm above the box 
        start = move_to_target(m,d,robot, start, id, gripper)

        # drop the box  the corresponding landing 
        start = pyramid(model=m, data=d,robot=robot, start=start, name=id, gripper=gripper)
    
   

    return robot.queue

# Note doing without inteprolation is too fast. You can see anything. It just chains up position. ---> need interpolation TRIAL with exercice 7
    
    
#  id = "box1"
#     start = robot.get_current_q()
#     gripper = 0
#     # move 10 cm above the box 
#     start = move_to_target(m,d,robot, start, id, gripper)

#         # move closely to the box 
#     start = move_close(m,d,robot, start, id, gripper)
#     print(id +" catched" )
#     gripper =  255
#     robot.set_gripper(value=gripper)
#     # move 10 cm above the box 
       
         
