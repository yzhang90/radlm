import numpy
from pylab import *
#m/s^2
A = 0.5
b = 0.5
#s
eps = 0.1
#m/s
vr = 1.0
#m
R_0 = 2.0

vr = numpy.arange(0.0,1.0,0.01)

sizes = []


for v in vr:
  dyn_win_size = (A/b+1)*(A/2*eps*eps+eps*v) + (1/(2*b))*v*v + R_0
  sizes.append(dyn_win_size)
  print "dyn_win_size = ", dyn_win_size

plot(vr, sizes)

xlabel('vr (m/s)')
ylabel('dynamic window size (m)')
title('')
#grid(True)
savefig("dwmonitor_plot.png")
show()


