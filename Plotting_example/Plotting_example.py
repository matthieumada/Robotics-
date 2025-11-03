import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.io import savemat, loadmat


## load trajectory from file 
Data = loadmat('example_data.mat')

# --- Access variables ---
Trj1 = Data['Jpos1']
Trj2 = Data['Jpos2']

fig, ax = plt.subplots(6, 1, figsize=(8, 10))  # make taller figure

joint_names = [f'Joint{i+1}' for i in range(6)]

for i in range(6):
    ax[i].plot(Trj1[:, i], color='b', label='Trj1')
    ax[i].plot(Trj2[:, i], color='r', linestyle='--', label='Trj2')
    ax[i].set_title(joint_names[i])
    ax[i].grid(True)
    ax[i].legend()

# save file in pdf for the project
plt.suptitle('Joint trajectory')
plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.savefig('Joint_pos.pdf', format='pdf')
plt.show()



