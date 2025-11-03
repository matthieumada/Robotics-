
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend


import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt
from robot import *

# quintinc only give movements fast and abrutp
# via_points only movement very slow
# combine both to have a smooth and fast movement

# Global queue container for trajectories
QUEUE = [] 

# Compute a polynomial from point A to B (abrupt and fast)
def quintic_poly(start_q, end_q, steps):
    # Create trajectory
    trajectories = [rtb.quintic(q0=start_q[i], qf=end_q[i], t=steps).q for i in range(6)]
    trajectories = np.array(trajectories).transpose()
    for step in trajectories:
        QUEUE.append((step, None))

# Transform the points for a smooth trajectory through via points ( to make trajectory smooth but slower)
def via_points(via_q, dt=0.002, tacc=1, qdmax=1):
    traj = rtb.mstraj(viapoints=via_q, dt=dt, tacc=tacc, qdmax=qdmax)
    for step in traj.s: # .s to get position information
        QUEUE.append((step, None))
    


def program(d, m):
    # Define our robot object
    robot = UR5robot(data=d, model=m)

    current_q = robot.get_current_q()

    object_list = ["pickup_point_box", "pickup_point_cylinder", "pickup_point_tblock", "pickup_point_cylinder", "pickup_point_box"]

    # Collect a list of target q-poses as your via points. 
    via_q = [current_q]
    q0 = current_q
    for obj in object_list:
        obj_frame = get_mjobj_frame(model=m, data=d, obj_name=obj) # Get body frame
        obj_frame = obj_frame * sm.SE3.Rx(-np.pi)
        obj_desired_q = robot.robot_ur5.ik_LM(Tep=obj_frame, q0=q0)[0] # compute the inverse kinematics to know how to get to the desired frame
        via_q.append(obj_desired_q)
        q0 = obj_desired_q

    via_q = np.array(via_q) # list of angular values for each joint 
    print(via_q)
    
    # Choose which trajectory method to use between via points, quintic polynomial or both combined 
    # Exercise 5.1: VIA POINTS
    via_points(via_q, dt=0.002, tacc=1.1, qdmax=25)

    # Exercise 5.2: QUINTIC POLY
    # for i in range(1, len(via_q)):
    #     quintic_poly(start_q=via_q[i-1], end_q=via_q[i], steps=500)

    # Both combined
    #first create an smooth movement to the via points
    trajectory_smooth = rtb.mstraj(viapoints=via_q, dt=0.002, tacc=1.1, qdmax=1)
    # then  accelerate each little movement between via points with quintinc polynomial 

    # QUEUE.clear() # reset QUEUE to avoid 2 trajectory in the same time 
    # for i in range(1, len(trajectory_smooth.s),500):
    #     QUEUE.append((trajectory_smooth.s[i-1], None) ) # add the starting via point
    #     quintic_poly(start_q=trajectory_smooth.s[i-1], end_q=trajectory_smooth.s[i], steps=100)

    # PLOT trajectory profiles
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

    plt.savefig("/home/delinm/Documents/Robotics--main/display/trajectory_profile_ex5_via_points.png")
    plt.show()

    # The calculated trajectory from the robot is sent to the controller
    return QUEUE
    

       
         