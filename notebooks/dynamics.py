#!/usr/bin/env python
# coding: utf-8

# In[1]:
import time
import sys

import numpy as np
import roboticstoolbox as rtb
from spatialmath import *
from math import pi
import matplotlib.pyplot as plt
from matplotlib import cm
np.set_printoptions(linewidth=100, formatter={'float': lambda x: f"{x:8.4g}" if x > 1e-10 else f"{0:8.4g}"})



# We will instantiate a model of the Puma 560 robot which has well known inertial parameters

# In[2]:


p560 = rtb.models.DH.Puma560()


# and show its configuration in a typical desktop working pose

# In[5]:


p560.plot(p560.qn, block=False)


# The rigid-body equations of motion for a robot are a set of coupled differential equations
# $$
# \mathbf{M}(\mathit{\ddot{q}}) \mathit{\ddot{q}} + \mathbf{C}(\mathit{q}, \mathit{\dot{q}}) \mathit{\dot{q}} + \mathbf{g}(\mathit{q}) = \mathit{\tau}
# $$
# which relate the motion of the robot $(\mathit{q}, \mathit{\dot{q}}, \mathit{\ddot{q}})$ and the applied torque $\mathit{\tau}$.  The coefficients in this equation are:
# - the inertia or mass matrix $\mathbf{M}(\mathit{\ddot{q}})$ which is a function of joint configuration
# - the centripetal and Coriolis or velocity term which is a function of joint configuration and rate
# - the gravity load which is a function of joint configuration
# 
# If the robot is not moving, that is $\mathit{q} = \mathit{\dot{q}} = 0$ then the equation becomes
# $$
# \mathbf{g}(\mathit{q}) = \mathit{\tau}
# $$
# where $\mathit{\tau}$ is the torque required for this condition $\mathit{q} = \mathit{\dot{q}} = 0$ to be true, that is, the torque required to stop the robot falling under its own weight.  The toolbox can compute this

# In[6]:

print('here 0', file=sys.stdout)
time.sleep(1)

p560.gravload(p560.qn)


# and it shows, as expected, that the shoulder is exerting significant torque to hold the arm up and stationary.
# 
# The inertia matrix relates torque to joint acceleration and is the mass in a multi-dimensional version of Newton's second law $F = m a$.  In this configuration the inertia matrix is

# In[7]:


p560.inertia(p560.qn)


# The diagonal elements $M_{jj}$ indicate the inertia experienced by the joint $j$, ie. Newton's second law for this joint is $\tau_j  = M_{jj} \ddot{q}_j$.
# 
# The matrix is symmetric and the off-diagonal terms $M_{ij} = M_{ji}$ couple acceleration of one joint into a disturbance torque on another joint, ie.  $\tau_j = M_{ij} \ddot{q}_i$.
# 
# The inertia matrix is a function of joint configuration, that is, the elements of the inertia matrix change as we vary the angles of joints 1 and 2, ie. $q_2$ and $q_3$.  It is this configuration varying inertia and coupling between joints that is a fundamental challenge for high-quality joint control.

# In[8]:


N = 100
(Q2, Q3) = np.meshgrid(np.linspace(-pi, pi, N), np.linspace(-pi, pi, N))
M11 = np.zeros((N,N))
M12 = np.zeros((N,N))
for i in range(N):
    for j in range(N):
        M = p560.inertia(np.r_[0, Q2[i,j], Q3[i,j], 0, 0, 0])
        M11[i,j] = M[0,0]
        M12[i,j] = M[0,1]


# The inertia "seen" by joint 1 varies as a function of $q_2$ and $q_3$ as shown below

# In[12]:


fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(Q2, Q3, M11, cmap=cm.coolwarm, linewidth=0, antialiased=False)
fig.colorbar(surf, shrink=0.9, aspect=10, pad=0.12)
ax.set_xlabel('$q_2$ (rad)')
ax.set_ylabel('$q_3$ (rad)')
ax.set_zlabel('$M_{11}$ ($kg.m^2$)')
plt.show(block=False)


# The ratio of maximum to minimum values is

# In[10]:


M11.max() / M11.min()


# The coupling inertia between joints 1 and 2 also varies with configuration and we can plot that as well

# In[11]:


fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(Q2, Q3, M12, cmap=cm.coolwarm, linewidth=0, antialiased=False)
fig.colorbar(surf, shrink=0.9, aspect=10, pad=0.12)
ax.set_xlabel('$q_2$ (rad)')
ax.set_ylabel('$q_3$ (rad)')
ax.set_zlabel('$M_{12}$ ($kg.m^2$)')
plt.show(block=False)


# In[3]:

print('here 1', file=sys.stdout)
time.sleep(1)



qd = np.r_[0, 1, 0, 0, 0, 0]

# BUG SEEMS TO BE IN NEXT LINE

p560.coriolis(p560.qn, qd) @ qd


# and we see that it exerts a torque on the waist and elbow joints.
# 
# The algorithms to compute the various terms in the rigid-body equations of motion are based on the recursive Newton-Euler algorithm

# In[4]:
for i in range(72):
    p560.rne(p560.qn, np.zeros((6,)), np.zeros((6,)), grav=[0,0,0])

p560.rne(p560.qn, np.zeros((6,)), np.zeros((6,)))

print('here 2', file=sys.stdout)
time.sleep(1)

# In[5]:

# p560nf = p560.nofriction()
# p560nf.delete_rne()

# print(p560nf)

tg = p560.fdyn(5, p560.qn, dt=0.05)

print('here 3', file=sys.stdout)
time.sleep(1)


# # The first line needs some explanation.  The Toolbox can model two types of joint friction:
# # - viscous friction which is linearly related to joint velocity
# # - Coulomb friction which is **non-linearly** related to joint velocity
# # 
# # Coulomb friction is a _harsh_ non-linearity and it causes the numerical integrator to take very small times steps, so the result will take many minutes to compute.  To speed things up, at the expense of some modeling fidelity, we set the Coulomb friction to zero, but retain the viscous friction.  The `nofriction()` method returns a clone of the robot with its friction parameters modified.
# # 
# # The computed joint configuration trajectory is

# # In[ ]:


# tg.q


# # which we can plot using a Toolbox convenience function

# # In[ ]:


rtb.tools.trajectory.qplot(tg.q, tg.t, block=False)


# # or we can animate it, showing the robot collapsing under gravity

# # In[ ]:


# p560.plot(tg.q.T)p560nf = p560.nofriction()

plt.pause(2)

print('done')



# The motion of the robot quickly dies out and it hangs downward, this loss of energy is due to the viscous friction in the robot's joints.
