import numpy as np
from spatialmath import UnitQuaternion
import matplotlib.pyplot as plt

def plot_q_interp(Qinterp):
    # input interpolated quaternion array
    # code from: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.geometric_slerp.html 
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, y, z, color='y', alpha=0.1)
    ax.plot(Qinterp.vec[...,0],Qinterp.vec[...,1],Qinterp.vec[...,2], c='k')


# quaternion
v1 = [0.7071, 0.7071,  0.0000,  0.0000]
v2 = [0.5000, -0.0000,  0.8660, -0.0000]

#Initial orientation 
Q1 = UnitQuaternion(v1)

# Final orientation
Q2 = UnitQuaternion(v2)
print("Q1.vec",Q1.vec, "Q1:", Q1)
print(Q2.vec)

# quaternion interpolation 
# Typically, calculate 100 interpolated quaternions between Q1 and Q2
QQ = Q1.interp(Q2,100)
plot_q_interp(QQ)
print("Interpolation Q1 and Q2", QQ.vec)

fig= plt.figure(2)
ax = fig.add_subplot(111, projection='3d')
Q1.plot(ax = ax, frame='A', color='green')
Q2.plot(ax = ax, frame='B', color='red')

# plot interpolated quaternion

plt.show()