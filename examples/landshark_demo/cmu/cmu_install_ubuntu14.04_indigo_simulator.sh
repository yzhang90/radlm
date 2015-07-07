#!/bin/sh

#Installing Git
#sudo apt-get install git #not necessary in ubuntu 14.04 and/or CMU-VM

#Sri Wiki
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full #this comes first in sri wiki
sudo apt-get install python3.4 #python3.4 is already the newest version.
sudo apt-get install libpopt-dev
sudo apt-get install libbullet-dev
sudo apt-get install ros-indigo-libg2o libsuitesparse-dev
sudo apt-get install libcrypto++9 #libcrypto* #need to check if all necessary
sudo apt-get install python-rosdep python-wstool build-essential
sudo rosdep init
sudo sh -c 'echo "yaml http://www.ai.sri.com/~aravind/ros/30-hacms.yaml" > /etc/ros/rosdep/sources.list.d/30-hacms.list'
rosdep update

#Navigation
sudo apt-get install ros-indigo-move-base 
sudo apt-get install ros-indigo-amcl
sudo apt-get install ros-indigo-robot-pose-ekf
sudo apt-get install ros-indigo-fake-localization
sudo apt-get install ros-indigo-robot-localization
sudo apt-get install ros-indigo-map-server
sudo apt-get install ros-indigo-sbpl #no indigo version, needs to be from source 

#simulator
wget http://www.cyberbotics.com/archive/linux/webots_7.1.2_amd64.deb
sudo apt-get -f install
sudo apt-get install libssl0.9.8
sudo dpkg -i webots_7.1.2_amd64.deb

#SRI Git Repo
git clone git@git.ai.sri.com:ros_groovy_base

#Standard
source /opt/ros/indigo/setup.bash
cd ./ros_groovy_base/landshark
rosdep install --from-paths src --ignore-src --rosdistro indigo -y

#manual changes
#git checkout CMU
#cp ./cmu/manual_changes/whatever ~/Desktop/some_folder
#git checkout master
#cp ~/Desktop/some_folder ./landshark/src/wherever

#standard
INSTALL_DIR=/opt/hacms/landshark_simulator
sudo mkdir -p ${INSTALL_DIR}
sudo chown `whoami`:users ${INSTALL_DIR}

touch ./src/landshark_navigation/CATKIN_IGNORE
rm ./src/landshark_sim/CATKIN_IGNORE

#catkin_make install -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}
catkin_make install -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCATKIN_BLACKLIST_PACKAGES="landshark_radar;landshark_base;landshark_robot"






