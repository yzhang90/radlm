#!/usr/bin/env python
# 
# Copyright (c) 2013 SpiralGen, Inc. 
# 
# The material contained in this release is copyrighted. It may not be copied, 
# reproduced, translated, reverse engineered, modified or reduced to any 
# electronic medium or machine-readable form without the prior written consent 
# of SpiralGen, Inc.
#
# Author(s): Jason Larkin (jason.larkin@spiralgen.com)
#            

#--------------------------------------------------------------------------------
import roslib
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String

import math
import sys
import threading
import re
import time

import pylab
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
from matplotlib import animation

#from ros_animate import Animate

accel_lin_max = 4.0
decel_lin_max = accel_lin_max
dt = 0.05
A = accel_lin_max
b = decel_lin_max
epsilon = dt
dt = 0.05
V = 0.0
num_vr = 100
vr = np.linspace(0.0,5.0,num_vr)
output1 = vr**2/(2.0*b) + vr*(V/b + epsilon*(A/b+1.0)) + (A/b+1.0)*(A/2.0*epsilon**2 + epsilon*V)
output2 = vr**2/(2.0*b) + V*vr/b + (A/b +1)*(A/2.0*epsilon**2 + epsilon*(vr+V))

plt.figure(1)
ax1 =plt.subplot(111)
plt.plot(vr[:],output1[:],'k',label='dwmonitor.c')
plt.plot(vr[:],output2[:],'ko',label='dynwind.pptx')

ax1.set_xlabel(r'$v_r$ (m/s)')
ax1.set_ylabel(r'safe distance (m)')
pylab.xlim([min(vr[:]),max(vr[:])])
pylab.ylim([min(output1[:]),max(output1[:])])
plt.legend(loc = 'upper left')

plt.show()
plt.close()


#	inputs:
#	D[0] = (A/b+1)*(A/2*eps^2+eps*V)
#	D[1] = V/b + eps*(A/b+1)
#	D[2] = 1/(2*b)

#	X[0] = vr
#	X[1..2] = pr.x, pr.y
#	x[3..4] = po.x, po.y

#	computes:
#	(A/b+1)*(A/2*eps^2+eps*V) + (V/b + eps*(A/b+1))*vr + (1/(2*b))*vr^2 < || pr - po||_oo









