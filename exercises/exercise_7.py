
import numpy as np
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt
import random

from ompl import base as ob
from ompl import geometric as og

from robot import *


class StateValidator:
    def __init__(self, d, m, num_joint):
        self.d = d
        self.m = m
        self.num_joint = num_joint
    
    def __call__(self, state):
        # print("isStateValid - state: ", state)
        q_pose = [state[i] for i in range(self.num_joint)]
        return is_q_valid(d=self.d, m=self.m, q=q_pose)

def rrt_plan(d, m, start_q, goal_q):
    num_joint = 6
    # Create your q-space (choose one of the above)
    space = ob.RealVectorStateSpace(num_joint)
    bounds = ob.RealVectorBounds(num_joint)
    bounds.setLow(-3.14)
    bounds.setHigh(3.14)
    space.setBounds(bounds)
    
    # Create SimpleSetup
    ss = og.SimpleSetup(space)
    # Create a partial function with bound arguments
    validator = StateValidator(d, m, num_joint)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(validator))
    
    # Set start and goal states
    start = ob.State(space)
    goal = ob.State(space)
    
    # Set specific joint values for start and goal
    for i in range(num_joint):
        start[i] = start_q[i] # initial joint angles
        goal[i] = goal_q[i]  # goal joint angles (~90 degrees)
    
    ss.setStartAndGoalStates(start, goal)

    # ===== EXPLICITLY SET PLANNER =====
    # Planner is RRT
    planner = og.RRT(ss.getSpaceInformation())
    ss.setPlanner(planner)
    
    # Solve the problem
    solved = ss.solve(10.0)
    
    if solved:
        # Get the solution path
        path = ss.getSolutionPath()
        print("Found a Solution!")
        # Print basic information about the path
        print(f"Path length: {path.length()}")
        print(f"Number of states: {path.getStateCount()}")

        solution_trajectory = []
        for i in range(path.getStateCount()):
            state = path.getState(i)
            q_pose = [state[i] for i in range(space.getDimension())]
            print(f"State {i}: {q_pose}")
            solution_trajectory.append(np.array(q_pose))

        return solution_trajectory
    
# option 1 iterative 
# version improved with AI
# def path_prune_simple(d, m, trajectory):
#     pruned = [trajectory[0]]
#     i = 0
#     while i < len(trajectory) - 1:
#         j = len(trajectory) - 1
#         # recule tant qu'on ne trouve pas de segment valide
#         while j > i and not check_valid_linear(d, m, trajectory[i], trajectory[j]):
#             j -= 1
#         pruned.append(trajectory[j])
#         i = j
#     return pruned

# without AI 
def path_prune_simple(d, m, trajectory):
    pruned = [trajectory[0]]
    # TODO: Implement your path pruning algorithm
    start = 0 
    # browse the list for the first point 
    for i in range(len(trajectory)-2):
        start += 1
        reacher = start +1 # set an reference for the closest reached 
        # browse the list from the reference + 1 to the last one 
        for final in range(start + 2 ,len(trajectory)):
            new_trajectory = rrt_plan(d, m, trajectory[start], trajectory[final])
            if new_trajectory : # new trajectory found
                if final > reacher: # change the reference 
                    reacher = final 
        pruned.append(trajectory[reacher])
    print("older trahectory:", trajectory)
    print("new trajectory found", pruned)
    return np.array(pruned)

# option 2 : dynamic without AI
def path_prune_dynamic(d, m, trajectory):
    indices = browser(d, m, trajectory, 1, [0])
    new_traj = []
    for j in indices : 
        new_traj.append(trajectory[j])
    print("Dynamic pruning indices:", indices)
    return new_traj


def browser(d, m, trajectory, reach, indices):
    start = indices[-1]
    last_valid = start

    # browse
    for j in range(start + 1, len(trajectory)):
        if check_valid_linear(d, m, trajectory[start], trajectory[j]):
            last_valid = j  # on peut aller directement jusqu’à j
        else:
            break  # collision found stop

    indices.append(last_valid)

    # end of path , mission commpleted 
    if last_valid >= len(trajectory) - 1:
        return indices

    # Sinon, on continue à partir du dernier point valide
    return browser(d, m, trajectory, last_valid, indices)

def check_valid_linear(d, m, a, b):
    # We check if it's possible to reach b from a (linear in joint space) without any collisions
    for qpose in np.linspace(a, b, 100):
        if not is_q_valid(d=d, m=m, q=qpose):
            return False
    return True


def program(d, m):

    # Simple program using RRT for collision free planning of a pick and place of the box
    # Use your path pruning algorithm to reduce redundant q-poses

    # Define our robot object
    robot = UR5robot(data=d, model=m)

    start_q = robot.get_current_q()
    # # Define grasping frames for object: box
    obj_frame = get_mjobj_frame(model=m, data=d, obj_name="box")
    obj_frame = obj_frame * sm.SE3.Rx(-np.pi)

    goal_q = robot.robot_ur5.ik_LM(Tep=obj_frame, q0=start_q)[0]
    sol_traj = rrt_plan(d=d, m=m, start_q=start_q, goal_q=goal_q)

    robot.move_j_via(points=sol_traj, t=500)   

    robot.set_gripper(255)

    start_q = goal_q
    # # Define grasping frames for object: box
    obj_frame = get_mjobj_frame(model=m, data=d, obj_name=f"drop_point_box")
    obj_frame = obj_frame * sm.SE3.Rx(-np.pi)

    goal_q = robot.robot_ur5.ik_LM(Tep=obj_frame, q0=start_q)[0]
    sol_traj = rrt_plan(d=d, m=m, start_q=start_q, goal_q=goal_q)
    new_sol_traj = path_prune_dynamic(d, m, sol_traj)
    print("Before: ", len(sol_traj))
    print("After: ", len(new_sol_traj))

    robot.move_j_via(points=new_sol_traj, t=500)   
    robot.set_gripper(0)

    return robot.queue

    
    

       
         
