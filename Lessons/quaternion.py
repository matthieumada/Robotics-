import spatialmath as sm
import numpy as np

""" This file is to remember how to use quaternions in spatial math"""

# firts define quaternion:
q_a = sm.Quaternion([1,1,0,0])
q_b = sm.Quaternion([2,0,-2*np.sqrt(3),0])
print("q_a: ", q_a, "\tq_b: ", q_b)

# Norm 
print("Norm of q_a: ", q_a.norm(), "\tNorm of q_b: ", q_b.norm())

# Normalize 
qa = q_a.unit() # UnitQuaternion() as well works
qb = q_b.unit()
print("Normalized qa: ", qa, "\tNormalized qb: ", qb)
print("Norm of normalized qa: ", qa, "\tNorm of normalized qb: ", qb)

print(" __________________________________")
print(" Quaternion operations")

#addition and subtraction
print("Addition: qa + qb = ", qa + qb)
print("Subtraction: qa - qb = ", qa - qb)

#multiplication
print("Multiplication: qa * qb = ", qa * qb)
print("Multiplication: qb * qa = ", qb * qa)

# ...existing code...

# Division (qa / qb) : équivalent à qa * qb.inverse()
print("Division: qa / qb = ", qa * qb.inv())

# Conjugué
print("Conjugate of qa: ", qa.conj())
print("Conjugate of qb: ", qb.conj())



# Inverse
print("Inverse of qa: ", qa.inv())
print("Inverse of qb: ", qb.inv())

rt_qb = sm.base.q2r(qb.vec)
print("Rotation matrix from qb: \n", rt_qb)

Q = sm.base.r2x(rt_qb)
print("Quaternion from rotation matrix: ", Q)


