
import numpy as np
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt
plt.style.use('seaborn-v0_8-whitegrid')

import plotly.graph_objects as go
from robot import *

"""
The version of this program use to generate a path with a task space interpolation. In that case the trajectory is  not smooth
and we use inverse kinematrics each step heavy compuattion and actuator not linearized but better control"""

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
        q_t = q0 + (t-t0)* (qf-q0) /T 
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
        if (t0<=t) and (t<t0 + tb):
            q_t = q0 + 0.5 * ddqb *(t-t0)**2

        elif ((t0 + tb)<=t) and (t< tf - tb):
            q_t = q0 + ddqb*tb*(t-t0-tb/2)


        elif tf-tb <=t and t<tf:
            q_t = qf - 0.5*ddqb*(tf - t)**2
        QUEUE.append((q_t,None))

def trapezoidal_trajectory(robot, via_q):
    # Trapezoidal interpolation with control of the speed by the number of steps 
    if robot.gripper_value > 0:
        t = 450
    else:
        t = 5
    print("Interpolate trajectory")
    via_q = np.array((via_q))
    L = np.shape(via_q)[0]
    for joint in range(1, L):
        traj = rtb.mtraj(tfunc=rtb.trapezoidal, q0=via_q[joint-1], qf=via_q[joint], t=t) # t=number of steps
        for step in traj.s: # .s to get position information:
            robot.queue.append((step, robot.gripper_value))
    return 

def quintic_poly(start_q, end_q, steps):
    # Create trajectory
    trajectories = [rtb.quintic(q0=start_q[i], qf=end_q[i], t=steps).q for i in range(6)]
    trajectories = np.array(trajectories).transpose()
    for step in trajectories:
        QUEUE.append((step, None))


def program(d, m):
    # global value 
    global QUEUE, velocity, acceleration
    # Define our robot object
    robot = UR5robot(data=d, model=m)

    current_frame = robot.get_current_tcp()
    current_q = robot.get_current_q()

    # object_list = ["pickup_point_box", "pickup_point_cylinder", "pickup_point_tblock", "pickup_point_cylinder", "pickup_point_box"]
    q0 = current_q
    t0 = current_frame
    new_frame = current_frame 
    path_linear = np.linspace(new_frame.t[2], new_frame.t[2]+0.15, 10)
    Angle = []
    for j in path_linear:
        new_frame.t[2] = j
        obj_desired_q = robot.robot_ur5.ik_LM(Tep=new_frame, q0=q0)[0]
        Angle.append(obj_desired_q)
        QUEUE.append((obj_desired_q, None))
        # parabolic_q_interpolation(start_q=q0, end_q=obj_desired_q, steps=10)
        q0 = obj_desired_q
    
    path_linear = np.linspace(new_frame.t[2], new_frame.t[2]-0.3, 300)
    for j in path_linear:
        new_frame.t[2] = j
        obj_desired_q = robot.robot_ur5.ik_LM(Tep=new_frame, q0=q0)[0]
        Angle.append(obj_desired_q)
        QUEUE.append((obj_desired_q, None))
        # parabolic_q_interpolation(start_q=q0, end_q=obj_desired_q, steps=10)
        q0 = obj_desired_q

    path_linear = np.linspace(new_frame.t[2], new_frame.t[2]+0.15, 300)
    for j in path_linear:
        new_frame.t[2] = j
        obj_desired_q = robot.robot_ur5.ik_LM(Tep=new_frame, q0=q0)[0]
        Angle.append(obj_desired_q)
        QUEUE.append((obj_desired_q, None))
        # parabolic_q_interpolation(start_q=q0, end_q=obj_desired_q, steps=10)
        q0 = obj_desired_q
    # End of path interpolation
    # trapezoidal_trajectory(robot, via_q=Angle)

    data = [q_pose[0] for q_pose, _ in QUEUE] # one joint value to plot
    joints = [q_pose for q_pose, _ in QUEUE]
    coordinates = []
    for j in joints:
        coordinates.append(robot.robot_ur5.fkine(j).t)

    coordinates = np.array(coordinates)
    print(f"coordinates shape: {coordinates.shape}")
    time = np.linspace(0, 0.002*np.shape(coordinates)[0], np.shape(coordinates)[0])
    fig, axs = plt.subplots(3)
    print(coordinates[1,0])
    fig.suptitle('Trajectory profiles')

    axs[0].plot(time,coordinates[:,0]*100)
    axs[1].plot(time,coordinates[:,1]*100)
    axs[2].plot(time,coordinates[:,2]*100)
    

    # axs[2].set_xlabel("")
    axs[0].set_ylabel("X Axis [cm]")
    axs[0].set_ylim(-0,80)
    axs[1].set_ylabel("Y Axis [cm]")
    axs[1].set_ylim(-50,50)
    axs[2].set_ylabel("Z Axis [cm]")
    axs[2].set_xlabel("Time [s]")

    plt.savefig("./TDK_bis/Figure/trajectory_z.png")

    velocity = np.ones((np.shape(coordinates)[0]-1, 3))
    for j in range(3):
        velocity[:,j] = np.diff(coordinates[:,j]) #

    fig, axs = plt.subplots(3)
    fig.suptitle('Speedprofiles')
    axs[0].plot(time[0:np.shape(coordinates)[0]-1],velocity[:,0]*100)
    axs[1].plot(time[0:np.shape(coordinates)[0]-1],velocity[:,1]*100)
    axs[2].plot(time[0:np.shape(coordinates)[0]-1],velocity[:,2]*100)
    axs[0].set_ylabel("X Axis [cm/S]")
    axs[0].set_ylim(-50,50)
    axs[1].set_ylabel("Y Axis [cm/s]")
    axs[1].set_ylim(-50,50)
    axs[2].set_ylabel("Z Axis [cm/s]")
    axs[2].set_xlabel("Time [s]")
    plt.savefig("./TDK_bis/Figure/velocity_z.png")

    # 3d plot to have a better visualization
    ax = plt.figure().add_subplot(projection='3d')
    ax.plot(xs=coordinates[:,0],ys = coordinates[:,1], zs = coordinates[:,2], label = 'position in 3d')
    ax.set_xlabel('X Axis [cm]')
    ax.set_ylabel('Y Axis [cm]')
    ax.set_zlabel('Z Axis [cm]')
    ax.set_ylim(-0.5, 0.5)
    ax.set_xlim(-0.5, 0.5)
    plt.savefig('./TDK_bis/Figure/trajectory_3d_z.png')

    fig = go.Figure(data=go.Scatter3d(x=coordinates[:,0], y=coordinates[:,1], z=coordinates[:,2], mode='lines'))
    fig.update_layout(scene=dict(
    xaxis=dict(title='X Axis', range=[0, 0.8]),
    yaxis=dict(title='Y Axis', range=[-0.5, 0.5]),  # Modifiez ici pour Y
    zaxis=dict(title='Z Axis' )),  # Modifiez ici pour Z
    title='3D Trajectory')
    fig.write_html('./TDK_bis/Figure/Trajectory_3d_z.html')
 
    #  Check of joint limits regarding braccio 
    joints = (180/np.pi)* np.array(joints)
    fig, axs = plt.subplots(3)

    axs[0].plot(time,joints[:,0])
    axs[1].plot(time,joints[:,1])
    axs[2].plot(time,joints[:,2])

    axs[0].set_ylabel("Joint shoulder [°]")
    axs[1].set_ylabel("Joint elbow 1 [°] ")    
    axs[2].set_ylabel("Joint elbow 2 [°]")
    axs[2].set_xlabel("Time [s]")

    plt.savefig("./TDK_bis/Figure/joint_profile_shoulder_z.png")
    
    # second joint
    fig, axs = plt.subplots(3)
    axs[0].plot(time, joints[:,3])
    axs[1].plot(time, joints[:,4])
    axs[2].plot(time, joints[:,5])

    axs[0].set_ylabel("Joint Wrist 1 [°]")
    axs[1].set_ylabel("Joint Wrist 2 [°]")    
    axs[2].set_ylabel("Joint Wrist 3 [°]")
    axs[2].set_xlabel("Time [s]")

    plt.savefig("./TDK_bis/Figure/joint_profile_wrist_z.png")

    # vibration checking 
    speed = np.ones((np.shape(joints)[0]-1, 6))
    for j in range(6):
        speed[:,j] = np.diff(joints[:,j]) # speed between each point of the trajectory 
    
    fig, axs = plt.subplots(3)
    axs[0].plot(time[0:np.shape(speed)[0]], speed[:,0])
    axs[1].plot(time[0:np.shape(speed)[0]], speed[:,1])
    axs[2].plot(time[0:np.shape(speed)[0]], speed[:,2])

    axs[0].set_ylabel("Speed shoulder [°/s]")
    axs[1].set_ylabel("Speed elbow 1 [°/s]")    
    axs[2].set_ylabel("Speed elbow 2 [°/s]")
    axs[2].set_xlabel("Time [s]")

    plt.savefig("./TDK_bis/Figure/speed_profile_shoulder_z.png")
    
    # second joint
    fig, axs = plt.subplots(3)
    axs[0].plot(time[0:np.shape(speed)[0]], speed[:,3])
    axs[1].plot(time[0:np.shape(speed)[0]], speed[:,4])
    axs[2].plot(time[0:np.shape(speed)[0]], speed[:,5])

    axs[0].set_ylabel("Speed Wrist 1 [°/s]")
    axs[1].set_ylabel("Speed Wrist 2 [°/s]")    
    axs[2].set_ylabel("SSpeed Wrist 3 [°/s]")
    axs[2].set_xlabel("Time [s]")
    plt.savefig("./TDK_bis/Figure/speed_profile_wrist_z.png")

    max_x = 0
    max_y = 0
    max_z = 0
    for j in coordinates[:,0]:
        if j > max_x:
            max_x = j
    print(f"Max x coordinate: {max_x} and starting x coordinate: {coordinates[0,0]}")
    print(f"delat x coordinate: {max_x - coordinates[0,0]}")

    for k in coordinates[:,1]:
        if k > max_y:
            max_y = k
    print(f"Max y coordinate: {max_y} and starting y coordinate: {coordinates[0,1]}")
    print(f"delta y coordinate: {max_y - coordinates[0,1]}")

    for m in range(299,len(coordinates[:,2]-1)):
        if coordinates[m,2] > max_z:
            max_z = coordinates[m,2]

    print(f"Max z coordinate: {max_z} and starting z coordinate: {coordinates[0,2]}")
    print(f"delta z coordinate: {max_z - coordinates[0,2]}")
    return QUEUE

       
         
