#========================================================================
#  This software is free: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License Version 3,
#  as published by the Free Software Foundation.
#
#  This software is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  Version 3 in the file COPYING that came with this distribution.
#  If not, see <http:#www.gnu.org/licenses/>.
#========================================================================
#!
 # \file    obstacleMsg.msg
 # \brief   obstacle message 
 #	    obstacle: 
 #		type: (0 -> lat/lon/alt) (1 -> x/y/z)
 # 		pointStamped: (x/lat y/lon z/alt)		
 # \author  Fatma Faruq 
 # \other   HACMS CMU Advisor Manuela Veloso
 #
##========================================================================
uint8 convertedToMeters = 0
uint8 latlon = 1
uint8 type
geometry_msgs/PointStamped coordinates
