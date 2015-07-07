#!/bin/bash 
depmod -a
LOGDIR=/var/log/hacms
logrotate -f /etc/logrotate.d/plant
mkdir -p $LOGDIR
cd $LOGDIR 
/opt/hacms/bin/radlinit > plant.log 2>&1

