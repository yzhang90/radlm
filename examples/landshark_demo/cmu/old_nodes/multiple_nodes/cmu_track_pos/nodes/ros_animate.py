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

import math
import signal
import sys
import threading
import re
import time

import numpy as np
import matplotlib.pyplot as plt
#from matplotlib import animation
#import threading
#--------------------------------------------------------------------------------
  
class Animate:
  def __init__(self):
    plt.ion()
    self.x = []
    self.y = []
    self.fig = plt.figure()
    self.ax = self.fig.add_subplot(111)
    self.line1, = self.ax.plot(self.x, self.y, 'r-')
#--------------------------------------------------------------------------------
  def update_plot(self,new_x_data,new_y_data):
    self.line1.set_xdata(new_x_data)
    self.line1.set_ydata(new_y_data)
    self.fig.canvas.draw()
#--------------------------------------------------------------------------------  

    
    
    
