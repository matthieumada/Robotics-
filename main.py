import queue
import mujoco
import numpy as np
import time
import os
import time
import mujoco
import mujoco.viewer
import glfw
import roboticstoolbox as rtb
import spatialmath as sm
import mujoco as mj
import spatialmath as sm
import spatialmath.base as smb
from spatialmath import SE3
from spatialmath.base import trinterp, trnorm
import matplotlib.pyplot as plt
import numpy as np
from typing import List
import roboticstoolbox as rtb
from PIL import Image


from robot import *
from cam import *
from xml_generator import xml_file
#from exercises.exercise_1 import program
#from exercises.exercise_2_sol import program
#from exercises.exercise_3 import program
#from exercises.exercise_4 import program
from exercises.exercise_5 import program
#rom exercises.exercise_unknown import program
# Supporting: Ubuntu 22-24, python 3.10 

if __name__ == "__main__":
    # Initialize OpenGL context first
    # mj.GLContext(max_width=1280, max_height=720)  # Adjust size as needed

    model_path = "scene.xml"  # Replace with your XML file

    #exo 3 
    #model path = xml_file()   # Generate the XML file for the scene

    #exo 5
    #model_path = "scene_grasp_track.xml"
    
    time_step = 0.002 # Defined in scene.xml 
    
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)

    key_queue = queue.Queue()
    cmd_queue = []

    with mujoco.viewer.launch_passive(model=m, 
                                      data=d, 
                                      key_callback=lambda key: key_queue.put(key)
                                      ) as viewer:
        
        # Home position for the scene
        target_pos = np.array([0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0])  # UR home position
        ur_set_qpos(data=d, q_desired=target_pos)
        hande_ctrl_qpos(data=d, gripper_value=0) # Open gripper


        sim_start = time.time()
        while time.time() - sim_start < 3.0:
            mujoco.mj_step(m, d)
            viewer.sync()
        
        cmd_queue = program(d, m)  # Run the program after the initial setup
        if cmd_queue is None:
            # return empty trajectory
            cmd_queue = []
        # In your main loop
        last_command_time = time.time()
        command_interval = 0.3  # seconds

        # get_rgb(m, d, renderer)
        # get_depth(m, d, renderer)
        # get_pointcloud(m, d, renderer)
        # show_pointcloud()

        print(" --- Launch Robotics Arm--- ")
        while viewer.is_running():
            # Get current time
            current_time = time.time()
            

            # mj_step can be replaced with code that also evaluates
            # a policy and applies a control signal before stepping the physics.
            mujoco.mj_step(m, d)
            
            # print("num cmd_queue: ", len(cmd_queue))
             # Check if it's time to execute a new command
            # if current_time - last_command_time >= command_interval and len(cmd_queue): # TODO: Speed up controller of UR5e
            if len(cmd_queue): # TODO: Speed up controller of UR5e
            # if len(cmd_queue):
                cmd_element, cmd_queue = cmd_queue[0], cmd_queue[1:]
                desired_cmd, gripper_value = cmd_element

                if isinstance(desired_cmd, np.ndarray) and not None:
                    #ur_ctrl_qpos(data=d, q_desired=desired_cmd) # Controls the robot, i.e., runs dynamics. (default)
                    ur_set_qpos(data=d, q_desired=desired_cmd) # Sets the robot to a position (forcefully), i.e., no dynamics. (useful for visualization)

                if gripper_value is not None:
                    hande_ctrl_qpos(data=d, gripper_value=gripper_value)

                # Update the last command time
                last_command_time = current_time

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()

        print( "--- Program ends ---" )