
import numpy as np
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt
import random

from ompl import base as ob
from ompl import geometric as og

from robot import *


def get_joint_range(m, num_joint):
    lows, highs = [], []
    joint_name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    for i in range(num_joint):
        joint_id = m.joint(joint_name[i]).id
        lows.append(m.jnt_range[joint_id][0])
        highs.append(m.jnt_range[joint_id][1])
    # define an adequate range toenable to reach the box:
    #  especially some joint must positive as shoulder lift joiint
    print("before lows:", lows)
    lows[1] = 0.0
    print("lows", lows)
    return lows, highs

class StateValidator:
    # This is mujoco specific, so I have implemented this for you
    def __init__(self, d, m, num_joint):
        self.d = d
        self.m = m
        self.num_joint = num_joint
    
    def __call__(self, state):
        print("isStateValid - state: ", state)
        q_pose = [state[i] for i in range(self.num_joint)]
        return is_q_valid(d=self.d, m=self.m, q=q_pose) 

def plan(d, m, start_q, goal_q):
    num_joint = 6
    
    space = ob.RealVectorStateSpace(num_joint) # Create a joint-space vector instead of a 2D space as in the example
    #TODO: Create joint bounds

    lows, highs = get_joint_range(m, num_joint)
    print("Lows range of joint:", lows)
    print("Highs range of joint:", highs)
    
    bounds = ob.RealVectorBounds(num_joint)
    for i in range(num_joint):
        bounds.setLow(i, lows[i])
        bounds.setHigh(i, highs[i])
    space.setBounds(bounds)
    print("Space bounds:", space.getBounds()) 


    #TODO: Create SimpleSetup and validator 
    # Create a state validity checker object
    validator = StateValidator(d, m, num_joint)
    if validator == None:
        print("No trajectory found!!!!!!")
        return None
    ss = og.SimpleSetup(space) # Create SimpleSetup
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(validator))
    
    # TODO: Set start and goal poses
    # Setting start states
    start = ob.State(space)
    for i,v in enumerate(start_q):
        print("start_q[", i, "] = ", v)
        start[i] = v

    # Setting goal states
    goal = ob.State(space)
    for i,v in enumerate(goal_q):
        print("goal_q[", i, "] = ", v)
        goal[i] = v

    ss.setStartAndGoalStates(start, goal)

    # ===== EXPLICITLY SET PLANNER =====
    # TODO: Create/set your planner type
    # RRT Connect planner
    #planner = og.RRTConnect(ss.getSpaceInformation())

    # PPM planner
    # find well path and optimized and reutilisable 
    # slow to build and need a gret many of sample
    planner = og.PRM(ss.getSpaceInformation())
    ss.setPlanner(planner)
    
    # Solve the problem
    solved = ss.solve(10.0)
    print("Solved:", solved)

    
    if solved:
        # TODO: Extract solution path
        ss.simplifySolution()
        path = ss.getSolutionPath()
        print("Found solution:\n", path)

        # TODO: Return the q-poses to solution_trajectory
        number = 100
        path.interpolate(number)
        solution_trajectory = []
        for state in path.getStates():
            q = []
            print("state: ", state)
            for j in range(num_joint):
                q.append(float(state[j]))
                solution_trajectory.append(q)
        return solution_trajectory


def program(d, m):
    # Define our robot object
    robot = UR5robot(data=d, model=m)
    # Define grasping frames for object: box
    #obj_frame = get_mjobj_frame(model=m, data=d, obj_name="drop_point_box") # Get body frame
    start_q = robot.get_current_q()
    print("Start q: ", start_q)

    # TODO: Plan a collision free trajectory from start_q to a q-pose representating the pose in obj_frame
    obj_frame = get_mjobj_frame(model=m, data=d, obj_name="pickup_point_box") # Get body frame
    obj_frame = obj_frame * sm.SE3.Rx(-np.pi) # adjust for gripper orientation
    obj_desired_q = robot.robot_ur5.ik_LM(Tep = obj_frame, q0 = start_q)[0]# compute the inverse kinematics to know how to get to the desired frame
    print("Object desired q: ", obj_desired_q)

    trajectory = plan(d,m, start_q, obj_desired_q)

    # Define grasping frames for object: box
    obj_frame = get_mjobj_frame(model=m, data=d, obj_name="pickup_point_box") *sm.SE3.Ry(-np.pi) # Get body frame
    # TODO: Plan a collision free trajectory from the new current q-pose to a q-pose representating the pose in obj_frame
    robot.queue = []
    for q in trajectory:
        robot.queue.append((np.array(q, dtype=np.float64), 0))
    # print(" robot queue : ", robot.queue[-1])
    # print("trajectory:", trajectory[-1])

    # print(" --- Grasp the box --- ")
    # --- Trial to bring the box up ---
    # catch the box by closing the gripper
    robot.queue.append((np.array(trajectory[-1], dtype=np.float64), 255))
    current_q = robot.queue[-1][0]
    obj_frame = obj_frame * sm.SE3.Tz(0.1) # move 
    obj_desired_q = robot.robot_ur5.ik_LM(Tep = obj_frame, q0 = current_q)[0]
    print("Inverse kinamtic trial:",robot.robot_ur5.ik_LM(Tep = obj_frame, q0 = current_q))# compute the inverse kinematics to know how to get to the desired frame
    trajectory = plan(d,m,current_q,obj_desired_q)
    for q in trajectory:
        robot.queue.append((np.array(q, dtype=np.float64), 255))
    return robot.queue

    
# for automated : # via_q = [start_q]
    # q0 = start_q
    # object_list = ["pickup_point_box", "pickup_point_cylinder", "pickup_point_tblock"]
    # for obj in object_list:
    #     obj_frame = get_mjobj_frame(model=m, data=d, obj_name=obj) # get body frames
    #     obj_frame = obj_frame * sm.SE3.Rx(-np.pi) # adjust for gripper orientation
    #     obj_desired_q = robot.robot_ur5.ik_LM(Tep=obj_frame, q0=q0)[0] # compute the invere kinematics to know how to get to the desired frame
    #     via_q.append(obj_desired_q) # add to via points the articulation values
    #     q0 = obj_desired_q # update 

    # goal_q = via_q[-1]
    # print("Via points: ", via_q)

       
         
