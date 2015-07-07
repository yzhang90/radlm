#!/bin/bash 
depmod -a
LOG=/var/log/cmu-`date +%Y%m%d_%H%M`.log.gz
su hacms -c /opt/hacms/etc/roslaunch.sh 2>&1 | gzip -9 -c > ${LOG} &
sleep 1s
#LOG=/var/log/plant-`date +%Y%m%d_%H%M`.log.gz
#/opt/hacms/bin/radlinit 2>&1 | gzip -9 -c > ${LOG} &
 
source /etc/bash.bashrc
source /opt/ros/indigo/setup.bash

LOG=/var/log/plant-`date +%Y%m%d_%H%M`.log
/opt/hacms/bin/radlinit 2>&1 > ${LOG} &

