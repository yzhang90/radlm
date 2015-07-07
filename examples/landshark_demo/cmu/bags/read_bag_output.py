#!/usr/bin/env python
#rosbag play facingdramaSquare.bag
#rostopic echo /landshark/gps/longitude > facingdramaSquare.gps.lon.txt

from pylab import *
import utm

array_gps_lat = []
array_gps_lon = []
array_gps_lat_cnt = []
array_gps_lon_cnt = []
utm_array_x=[]
utm_array_y=[]
utm_origin_x=0.0
utm_origin_y=0.0

with open("facingmorewoodSquare.gps.lat.txt") as f:
  for idx,line in enumerate(f):
    if not line.startswith("---"):
      array_gps_lat.append(float(line))
      array_gps_lat_cnt.append(idx)  
      
#      print "line = ", line

with open("facingmorewoodSquare.gps.lon.txt") as f:
  for idx,line in enumerate(f):
    if not line.startswith("---"):
      array_gps_lon.append(float(line))
      array_gps_lon_cnt.append(idx)

for idx,num in enumerate(array_gps_lon):
  utm_tuple = utm.from_latlon(array_gps_lat[idx], array_gps_lon[idx])
  utm_array_x.append(utm_tuple[0])
  utm_array_y.append(utm_tuple[1])



#plt.subplot(2, 1, 1)
#plot(array_gps_lat_cnt, array_gps_lat)
#xlabel('time (cnts)')
#ylabel('gps lat (rads?)')
#title('facingdramaSquare landshark gps lat ')
#grid(True)
##savefig("test.png")

#plt.subplot(2, 1, 2)
#plot(array_gps_lon_cnt, array_gps_lon )
#xlabel('time (cnts)')
#ylabel('gps lon (rads?)')
#title('facingdramaSquare landshark gps lat ')
#grid(True)

plot( [x - utm_array_x[0] for x in utm_array_x], [y - utm_array_y[0] for y in utm_array_y])
xlabel('x (m)')
ylabel('y (m)')
savefig("facingdramaSquare.gps.xy.pdf")
show()
