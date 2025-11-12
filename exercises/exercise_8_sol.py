import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from robot import *
from typing import List, Union, Tuple
from exercises.dmp.canonical_system import CanonicalSystem
from exercises.dmp.dmp_joint import JointDMP
from exercises.dmp.dmp_position import PositionDMP

# You might need to install the following libraries
# pip3 install numpy numpy-quaternion matplotlib pandas

def from_q_to_pose(robot, qtrj):
    poses = [robot.robot_ur5.fkine(q) for q in qtrj]
    np_poses = np.array(poses)
    pos = np_poses[:, 0:3, 3]

    # Use the original orientation
    # but you need a QuaternionDMP object to handle the orientation part of the DMP. See "dmp_quaternion.py"
    # ori = np.array([sm.SO3(np_poses[i, 0:3, 0:3]) for i in range(len(np_poses))]) 

    # Keep the starting orientation
    start_ori  = robot.get_current_tcp().R
    ori = np.array([start_ori for i in range(len(np_poses))])
    return pos, ori, poses

def load_poses(robot):
    demo_filename = "exercises/dmp/recording.csv"
    demo = pd.read_csv(demo_filename)
    qtrj = demo.to_numpy()
    pos, ori, poses = from_q_to_pose(robot, qtrj)
    return pos, ori, poses

def program(d, m):
    print("Learning from Demonstration exercise")

    # Define our robot object
    robot = UR5robot(data=d, model=m)
    pos, ori, poses = load_poses(robot)

    # Discard first few sample due to inactive movement
    pos = pos[30:]
    ori = ori[30:]
    
    dt = 1/20
    # calculate time (tau) = to length of the trajectory
    tau = len(pos) * dt
    ts = np.arange(0, tau, dt)
    cs_alpha = -np.log(0.0001)

    ## encode DMP 
    dmp_q = PositionDMP(n_bfs=100, alpha=48, beta=12, cs_alpha=cs_alpha)

    ## Encode the dmp e.g., train 
    dmp_q.train(pos, ts, tau)

    ################## Change goal and start position ###################
    start_frame = get_mjobj_frame(model=m, data=d, obj_name="pickup_point_tblock") * sm.SE3.Rx(-np.pi) * sm.SE3.Tz(0.15) * sm.SE3.Tx(-0.15)  # Get body frame
    end_frame = get_mjobj_frame(model=m, data=d, obj_name="pickup_point_box") * sm.SE3.Rx(-np.pi) * sm.SE3.Tz(0.05) * sm.SE3.Tx(0.05)  # Get body frame

    new_p0 = pos[0] + [0, 0, 0]                 #start_frame.t  # Selected frame as new start position 
    new_gp = pos[-1] + [-0.25, 0.25, 0.2]       #end_frame.t    # Selected frame as new goal position

    dmp_q.set_trained(w=dmp_q.w, c=dmp_q.c, h=dmp_q.h, y0=new_p0, g=new_gp)
    ###########################################################################

    # ## integrate DMP
    p, dp, ddp = dmp_q.rollout(ts, tau)
    
    # plot position response 
    plt.figure(1)
    plt.plot(p, 'b')
    plt.plot(pos, 'r')
    plt.legend(['DMP traj', 'Orig traj'])
    plt.savefig("DMP_trajectory.png")
    print("Saved plot!")

    q_init = robot.get_current_q()

    for i in range(len(p)):
        # put position and orientation together to form a pose 
        dmp_pose = sm.SE3()
        dmp_pose.t = p[i]
        dmp_pose.R = ori[0]

        q_pose = robot.robot_ur5.ik_GN(dmp_pose, q0=q_init)[0]
        for _ in range(10):
            robot.queue.append((q_pose, None))
        q_init = q_pose

    ur_set_qpos(data=d, q_desired=robot.queue[0][0]) # Sets the robot to a position (forcefully), i.e., no dynamics. (useful for visualization)
    return robot.queue

    