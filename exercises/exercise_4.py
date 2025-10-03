
import numpy as np
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt

from robot import *


# Global queue container for trajectories

QUEUE = []


def linear_q_interpolation(start_q, end_q, steps):
    global QUEUE, velocity, acceleration
    q0 = start_q
    qf = end_q
    t0 = 0
    tf = 1
    T = tf-t0
    i = 0
    for t in np.linspace(t0, tf, steps):
        q_t = q0 + t* (qf-q0) /T 
        QUEUE.append((q_t,0)) # qpose and gripper value
    


def parabolic_q_interpolation(start_q, end_q, steps):
    q0 = start_q
    qf = end_q
    t0 = 0
    tf = 1
    td = 0.2# duration of travel
    tb = 0.8# blend time

    # Calculate required acceleration for the blend
    ddqb = 0.2 # constant acceleration during blend

    for t in np.linspace(t0, tf, steps):
        # acceleration phase
        if t<td:
            q_t =q0 +0.5*ddqb*t**2
            QUEUE.append((q_t, None))
            velocity.append(ddqb*t)
            acceleration.append(ddqb)
        # constant velcoity phase
        elif td<=t<tb:
            q_t = q_t + velocity[-1]*(t-td)
            QUEUE.append((q_t, None))
            velocity.append(velocity[-1])
            acceleration.append(0)
        # deceleration phase
        elif tb<=t<=tf:
            q_t = qf - 0.5*ddqb*(t - tb)**2
            QUEUE.append((q_t, None))
            velocity.append(velocity[-1] - ddqb*(t-tb))
            acceleration.append(-ddqb)


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
        # reset QUEUE for each object
        QUEUE = [] 
        velocity = np.array([])
        acceleration = np.array([])
        # # Define grasping frames for object: box
        obj_frame = get_mjobj_frame(model=m, data=d, obj_name=obj) # Get body frame
        obj_frame = obj_frame * sm.SE3.Rx(-np.pi)

        obj_desired_q = robot.robot_ur5.ik_LM(Tep=obj_frame, q0=q0)[0]

        # STEP 1: Implement your own linear-q interpolation
        linear_q_interpolation(start_q=q0, end_q=obj_desired_q, steps=300)

        # STEP 2: Implement your own parabolic-q interpolation
        #parabolic_q_interpolation(start_q=q0, end_q=obj_desired_q, steps=300)
        
        q0 = obj_desired_q
        t0 = obj_frame

    data = []
    for q_pose, gripper_value in QUEUE:
        # extract q_pose to data for plotting
        data.append(q_pose[0]) # y-value

    t = np.linspace(0,1,len(data))
    # Plot trajectory profile, position, velocity, accelaration
    plt.figure()
    
    QUEUE = np.array(QUEUE[])
    velocity = np.diff(QUEUE)
    acceleration = np.diff(velocity)
    # Position
    print(len(data))
    plt.subplot(3, 1, 1)
    plt.plot(t, data, label='position')
    plt.title("Position")
    plt.ylabel("Position")
    plt.xlabel("Time [s]")
    plt.legend()

    # Velocity
    print(len(velocity))
    plt.subplot(3, 1, 2)
    plt.plot(t, velocity, label='velocity')
    plt.title("Speed")
    plt.ylabel("Speed")
    plt.xlabel("Time [s]")
    plt.legend()

    # 3. Accélération
    print(len(acceleration))
    plt.subplot(3, 1, 3)
    plt.plot(t, acceleration, label='acceleration')
    plt.title("Acceleration")
    plt.xlabel("Time [s]")
    plt.ylabel("Accélération")
    plt.legend()

    plt.tight_layout()
    plt.savefig("/home/delinm/Documents/Robotics_Computer_Vision/Robotics/exercises/trajectory_profile_last_object_ex4.png")
    plt.show(block=False)
    plt.pause(5)
    plt.close()

    # The calculated trajectory from the robot is sent to the controller
    return QUEUE
    

       
         
