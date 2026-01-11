import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import matplotlib
# On utilise Qt5 au lieu de Tk
matplotlib.use('Qt5Agg') 

import matplotlib.pyplot as plt

### two link scara mechanism workspace representation 
# link lengths  m
l1 = 1
l2 = 1.5
# number of samples
samp = 100
# all possible theta values
theta1 = np.linspace(0,210,samp)*np.pi/180
theta2 = np.linspace(0,180,samp)*np.pi/180
# generate a grid of theta values 
th1,th2 = np.meshgrid(theta1,theta2, indexing='ij')
# functions to calculate the x and y coordinates for four links with variable angles 
X= l1*np.cos(th1)+l2*np.cos(th1+th2)
Y= l1*np.sin(th1)+l2*np.sin(th1+th2)
#plot results
plt.figure(1)
plt.title('2 link SCARA mechanism workspace')
plt.plot(X,Y,'b.')
plt.ylabel('y')
plt.xlabel('x')


# 3 link mechanism workspace representation 
l11 = 1
l22 = 1.5
l33 = 1.25
# number of samples
samp = 10
# all possible theta values
theta11 = np.linspace(-45,130,samp)*np.pi/180
theta22 = np.linspace(0,90,samp)*np.pi/180
theta33 = np.linspace(0,80,samp)*np.pi/180
# generate a grid of theta values 
th11,th22,th33 = np.meshgrid(theta11,theta22,theta33, indexing='ij')
# functions to calculate the x and y coordinates for four links with variable angles 
X1= l11*np.cos(th11) + l22*np.cos(th11+th22) + l33*np.cos(th11+th22+th33)
Y1= l11*np.sin(th11) + l22*np.sin(th11+th22) + l33*np.sin(th11+th22+th33)

#plot results
plt.figure(2)
plt.title('3 link mechanism workspace') 
plt.plot(X1.flatten(),Y1.flatten(),'b.')
plt.ylabel('y')
plt.xlabel('x')
plt.show()

# 4 link mechnaism
l1 = 2
l2 = 1.75
l3 = 1.25
l4 = 0.75

samp = 10
theta1 = np.linspace(0,130,samp)*np.pi/180
theta2 = np.linspace(0,90,samp)*np.pi/180
theta3 = np.linspace(0,80,samp)*np.pi/180
theta4 = np.linspace(0,40,samp)*np.pi/180

th1,th2,th3,th4 = np.meshgrid(theta1,theta2,theta3,theta4, indexing='ij')
X1= l1*np.cos(th1) + l2*np.cos(th1+th2) + l3*np.cos(th1+th2+th3) + l4*np.cos(th1+th2+th3+th4)
Y1= l1*np.sin(th1) + l2*np.sin(th1+th2) + l3*np.sin(th1+th2+th3) + l4*np.sin(th1+th2+th3+th4)

#plot results
plt.figure(3)
plt.title('4 link mechanism workspace') 
plt.plot(X1.flatten(),Y1.flatten(),'b.')
plt.ylabel('y')
plt.xlabel('x')
plt.show()