import numpy as np
from spatialmath import SO3, UnitQuaternion
import matplotlib.pyplot as plt

#Euler [Z,Y,X]
A1 = [45, 10,0] # °
A2 = [0, -80, 45] #°

# Matrix of rotation 
print("\n--- With  transformation matrix ---")

R1 = SO3.RPY(A1, unit='deg', order='zyx')
R2 = SO3.RPY(A2, unit='deg', order='zyx')

# from A1 to A2
R_rel = R2 * R1.inv()

print("R1=",R1)
print("R2=",R2)
print("final R", R_rel)

# quaternion
Q1 = UnitQuaternion(R1)
Q2 = UnitQuaternion(R2)

# Quaternion rotation 
Q_rel = Q2 * Q1.inv()
print("Q1=",Q1)
print("Q2=",Q2)
print("final Q", Q_rel, "not commutative",Q1 * Q2.inv())

print("\nVCheckcing  (Q_rel matrix :\n", Q_rel.R)
print("difference :", np.linalg.norm(Q_rel.R - R_rel.R))


## d. Interpolation (SLERP) et Plot
print("\n--- d. Interpolation Path (10 samples) ---")
traj = Q1.interp(Q2, 10) # traj contient 10 quaternions

# Affichage des valeurs
for i, q in enumerate(traj):
    print(f"Sample {i+1}: {q.vec}")

# Initialisation de la figure 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for q in traj:
    # On trace chaque frame. 
    # 'length' contrôle la taille des flèches
    q.plot(ax=ax, color='blue', alpha=0.5, length=0.2)

# Astuce visuelle : Tracer la ligne noire reliant les bouts des quaternions
# traj.vec est un tableau (10, 4), on prend x, y, z (colonnes 1, 2, 3)
vecs = traj.vec 
ax.plot(vecs[:, 1], vecs[:, 2], vecs[:, 3], 'k-', label='Quaternion Path')

# Réglages du graphique
ax.set_xlabel('qx')
ax.set_ylabel('qy')
ax.set_zlabel('qz')
plt.title("Interpolated Orientation Path on Sphere")
plt.legend()
plt.show()
