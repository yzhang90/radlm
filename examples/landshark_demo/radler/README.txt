
*** To get RADL working on a clean version of Ubuntu 14.04, run the
following commands: ***

sudo apt-get install cmake

wget https://bootstrap.pypa.io/get-pip.py && sudo python3.4 get-pip.py

sudo pip install tarjan

sudo pip install ipython

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'

wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install ros-indigo-ros-base


(echo ; echo "# Setup for ROS" ; echo "source /opt/ros/indigo/setup.bash" ) >> ~/.bashrc

source ~/.bashrc



*** To compile the Kestrel kernel modules on a test system run these commands: ***

cd radler/kestrel_lib/kmods

make

sudo mkdir -p /lib/modules/`uname -r`/kernel/extras

sudo cp *.ko /lib/modules/`uname -r`/kernel/extras

sudo depmod



*** To compile and run RADL file foo.radl (e.g., in the examples/good directory): ***

mkdir -p /tmp/foo/src

./radler.sh --ws_dir /tmp/foo/src compile path/to/foo.radl --plant foo.plant

cd /tmp/foo

catkin_make

for i in ./devel/lib/radlplant_plant/*; do sudo $i; done

** This will start the RADL nodes running; you can watch the output of node
   "node1" by executing the following (type control-C to stop):

tail -f foo.node1.log

** You can stop all the RADL nodes by typing:

sudo killall -r radlinit
