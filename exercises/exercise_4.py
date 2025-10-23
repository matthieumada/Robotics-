
import numpy as np
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt

from robot import *


# Global queue container for trajectories

QUEUE = []
box_pick1 = []
box_pick2 = []
cylinder_pick1 = []
cylinder_pick2 = []
tblock_pick = []


def linear_q_interpolation(start_q, end_q, steps):
    global QUEUE, velocity, acceleration
    q0 = start_q
    qf = end_q
    t0 = 0
    tf = 1
    T = tf-t0
    # Calculate of pisition with linear interpolation
    for t in np.linspace(t0, tf, steps):
        q_t = q0 + t* (qf-q0) /T 
        QUEUE.append((q_t,None)) # qpose and gripper value
    


def parabolic_q_interpolation(start_q, end_q, steps):
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
        if (t0<=t) and (t<t0*tb):
            q_t = q0 + 0.5 * ddqb *(tf-t0)**2

        elif ((t0 + tb)<=t) and (t< tf -tb):
            q_t = q0 + ddqb*tb*(t-t0-tb/2)
 

        elif tf-tb <=0 and t<=tf:
            q_t = qf - 0.5*ddqb*(tf - t)**2


def program(d, m):
    # global value 
    global QUEUE, velocity, acceleration
    # Define our robot object
    robot = UR5robot(data=d, model=m)

    current_frame = robot.get_current_tcp()
    current_q = robot.get_current_q()

    object_list = ["pickup_point_box", "pickup_point_cylinder", "pickup_point_tblock", "pickup_point_cylinder", "pickup_point_box"]
    q0 = current_q
    t0 = current_frame
    
    for obj in object_list:

        # # Define grasping frames for object: box
        obj_frame = get_mjobj_frame(model=m, data=d, obj_name=obj) # Get body frame
        obj_frame = obj_frame * sm.SE3.Rx(-np.pi)

        obj_desired_q = robot.robot_ur5.ik_LM(Tep=obj_frame, q0=q0)[0]

        # STEP 1: Implement your own linear-q interpolation
        linear_q_interpolation(start_q=q0, end_q=obj_desired_q, steps=300)

        # STEP 2: Implement your own parabolic-q interpolation
        parabolic_q_interpolation(start_q=q0, end_q=obj_desired_q, steps=300)
        
        q0 = obj_desired_q

    data = [q_pose[0] for q_pose, _ in QUEUE] # one joint value to plot

    fig, axs = plt.subplots(4)
    fig.suptitle('Trajectory profiles')

    pos = np.array(data)
    vel = np.diff(pos)
    acc = np.diff(vel)
    jerk = np.diff(acc)

    axs[0].plot(pos)
    axs[1].plot(vel)
    axs[2].plot(acc)
    axs[3].plot(jerk)

    # axs[2].set_xlabel("")
    axs[0].set_ylabel("Position")
    axs[1].set_ylabel("Velocity")
    axs[2].set_ylabel("Accelaration")
    axs[3].set_ylabel("Jerk")

    plt.savefig("trajectory_profile.png")

    # The calculated trajectory from the robot is sent to the controller
    return QUEUE

       
         
