import numpy as np
import os 
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
    print("Position:", pos)
    print("Orientation", ori)
    print("Poses:",poses)

    # Discard first few sample due to inactive movement
    pos = pos[30:]
    ori = ori[30:]
    
    dt = 1/20
    # calculate time (tau) = to length of the trajectory
    tau = len(pos) * dt
    ts = np.arange(0, tau, dt)
    cs_alpha = -np.log(0.0001)

    # TODO: Make a DMP object (position only) 
    DMP = PositionDMP()

    # TODO: Train the DMP on the loaded trajectory
    DMP.train(pos, ts, tau)

    # TODO: 1) Run the DMP with the original start and goal pose
    #p_reproduce, _, _ = DMP.rollout(ts, tau)

    # TODO: 2) (Afterwards) Change start and goal pose and re-run
    # new_start = pos[0] + np.array([0.1, 0.0, 0.0])
    # new_goal = pos[-1] + np.array([0.2, 0.0, 0.1])
    # DMP.set_trained(w=DMP.w, c = DMP.c, h=DMP.h, y0 = new_start, g=new_goal)
    #p_generalize, _, _ = DMP.rollout(ts, tau)

    # TODO: make a dmp rollout to get the dmp positions/trajectory
    p, dp, ddp = DMP.rollout(ts, tau)
    # TODO: plot position response and original trajectory
    plt.figure()
    plt.subplot(3,1,1)
    plt.plot(pos[:,0], label='Demo X')
    plt.plot(p[:,0], '--', label='DMP X')
    plt.legend(); plt.title('Position X')

    plt.subplot(3,1,2)
    plt.plot(pos[:,1], label='Demo Y')
    plt.plot(p[:,1], '--', label='DMP Y')
    plt.legend(); plt.title('Position Y')

    plt.subplot(3,1,3)
    plt.plot(pos[:,2], label='Demo Z')
    plt.plot(p[:,2], '--', label='DMP Z')
    plt.legend(); plt.title('Position Z')

    plt.tight_layout()
    out_dir = "display"                       # folder
    out_path = os.path.join(out_dir, "new_trajectory_generalize_exo8.png")
    plt.savefig(out_path)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(pos[:,0], pos[:,1], pos[:,2], label='Demo Trajectory', linewidth=2)
    ax.plot(p[:,0], p[:,1], p[:,2], '--', label='DMP reproduction')
    ax.legend()
    out_dir = "display"                       # folder
    out_path = os.path.join(out_dir, "new_trajectory3D_genzrealize_exo8.png")
    plt.savefig(out_path)

    #p = p_generalize  #DMP positions/trajectory

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

    ur_set_qpos(data=d, q_desired=robot.queue[0][0]) # Sets the robot to a position (forcefully), i.e., no dynamics.
    return robot.queue

    