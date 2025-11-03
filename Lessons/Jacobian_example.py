import roboticstoolbox as rtb
import numpy as np
import swift
import spatialmath as sm


Joint_pos = [
        [-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0] ,
        [-np.pi / 2, -np.pi / 2, -np.pi / 2, 0, np.pi / 2, 0],
        [np.deg2rad(0), np.deg2rad(-30),np.deg2rad(0),np.deg2rad(30),np.deg2rad(30),np.deg2rad(0)],
        [np.deg2rad(0), np.deg2rad(-120),np.deg2rad(120),np.deg2rad(-90),np.deg2rad(0),np.deg2rad(0)],
        [np.deg2rad(0), np.deg2rad(-45),np.deg2rad(-90),np.deg2rad(0),np.deg2rad(0),np.deg2rad(0)],
]

# Caluclate the Robots forward kinematics
TOOL_LENGTH = 0.15
robot_ur5 = rtb.DHRobot([
        rtb.RevoluteDH(d=0.1625, alpha=np.pi / 2.0, qlim=(-np.pi, np.pi)),              # J1
        rtb.RevoluteDH(a=-0.425, qlim=(-np.pi, np.pi)),                                 # J2
        rtb.RevoluteDH(a=-0.3922, qlim=(-np.pi, np.pi)),                                # J3
        rtb.RevoluteDH(d=0.1333, alpha=np.pi / 2.0, qlim=(-np.pi, np.pi)),              # J4
        rtb.RevoluteDH(d=0.0997, alpha=-np.pi / 2.0, qlim=(-np.pi, np.pi)),             # J5
        rtb.RevoluteDH(d=0.0996 + TOOL_LENGTH, qlim=(-np.pi, np.pi)),                   # J6
        ], name="UR5", base=sm.SE3.Rz(-np.pi))

# calculate Jacobian in Base frame with the starting angle 
jac= robot_ur5.jacob0(Joint_pos[2])

# calculate the determinant of the jacobian 
det = np.linalg.det(jac)
# check jacobian singularity 
print('Jacobian=',jac)
print('Jacobian Determinant =', det)
print('Is the jacobian singular?')
rtb.jsingu(jac) #displau the singulariy if nothing no singularity 
input("Press Enter to continue...")




