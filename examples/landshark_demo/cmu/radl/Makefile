WS = catkin_ws
RADLER = ../../radler/radler.sh

OBJS = constants.radlo arrays.radlo topics.radlo landshark.radlo ocu.radlo hacms.radlo comm.radlo test_base.radlo global.radlo debug.radlo devices.radlo cmu.radlo sim_gateway.radlo 

topics_DEPS = global.radlo arrays.radlo
landshark_DEPS = global.radlo arrays.radlo constants.radlo topics.radlo devices.radlo
ocu_DEPS = global.radlo arrays.radlo constants.radlo topics.radlo 
comm_DEPS = global.radlo arrays.radlo constants.radlo topics.radlo ocu.radlo
hacms_DEPS = global.radlo arrays.radlo constants.radlo topics.radlo devices.radlo landshark.radlo ocu.radlo comm.radlo

cmu_DEPS = arrays.radlo topics.radlo constants.radlo global.radlo
#cmu_sim_DEPS = arrays.radlo topics.radlo constants.radlo global.radlo
sim_gateway_DEPS = arrays.radlo topics.radlo constants.radlo global.radlo cmu.radlo

debug_DEPS = global.radlo arrays.radlo constants.radlo topics.radlo 
test_base_DEPS = global.radlo arrays.radlo constants.radlo topics.radlo ocu.radlo comm.radlo 

hacms_FLAGS = --plant plant
test_base_FLAGS = --plant plant

.PHONY: build sri cmu penn etc clean cleanall uninstall \
	cmu_nodes

all: cmu_nodes

debug:
	$(MAKE) debug.radlo
	$(MAKE) build

cmu_nodes:      
	$(MAKE) cmu.radlo
	echo 'SET_SOURCE_FILES_PROPERTIES(' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo `ls catkin_ws/src/radlast_*_cmu/user_src/src/monitor_dw/src/dwmonitor2.c | head | sed -e 's/catkin_ws.src/..\/../'` >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo 'PROPERTIES' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo 'COMPILE_FLAGS "-std=gnu99 -pedantic-errors -msse4.2 -Wa,-ahl=dwmonitor2.s"' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo ')' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo 'SET_SOURCE_FILES_PROPERTIES(' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo `ls catkin_ws/src/radlast_*_cmu/user_src/src/monitor_dw/src/dwmonitor.c | head | sed -e 's/catkin_ws.src/..\/../'` >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo 'PROPERTIES' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo 'COMPILE_FLAGS "-std=gnu99 -pedantic-errors -msse4.2 -Wa,-ahl=dwmonitor.s"' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo ')' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo 'SET_SOURCE_FILES_PROPERTIES(' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo `ls catkin_ws/src/radlast_*_cmu/user_src/src/monitor_ftest/ftest.c | head | sed -e 's/catkin_ws.src/..\/../'` >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo 'PROPERTIES' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo 'COMPILE_FLAGS "-std=gnu99 -pedantic-errors -msse4.2 -Wa,-ahl=ftest.s"' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo ')' >> $(WS)/src/ros/cmu/CMakeLists.txt
	$(MAKE) build

cmu_nodes_sim:     
	$(MAKE) cmu.radlo
	$(MAKE) sim_gateway.radlo
	echo 'SET_SOURCE_FILES_PROPERTIES(' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo `ls catkin_ws/src/radlast_*_cmu/user_src/src/monitor_dw/src/dwmonitor2.c | head | sed -e 's/catkin_ws.src/..\/../'` >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo 'PROPERTIES' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo 'COMPILE_FLAGS "-std=gnu99 -pedantic-errors -msse4.2 -Wa,-ahl=dwmonitor2.s"' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo ')' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo 'SET_SOURCE_FILES_PROPERTIES(' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo `ls catkin_ws/src/radlast_*_cmu/user_src/src/monitor_dw/src/dwmonitor.c | head | sed -e 's/catkin_ws.src/..\/../'` >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo 'PROPERTIES' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo 'COMPILE_FLAGS "-std=gnu99 -pedantic-errors -msse4.2 -Wa,-ahl=dwmonitor.s"' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo ')' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo 'SET_SOURCE_FILES_PROPERTIES(' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo `ls catkin_ws/src/radlast_*_cmu/user_src/src/monitor_ftest/ftest.c | head | sed -e 's/catkin_ws.src/..\/../'` >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo 'PROPERTIES' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo 'COMPILE_FLAGS "-std=gnu99 -pedantic-errors -msse4.2 -Wa,-ahl=ftest.s"' >> $(WS)/src/ros/cmu/CMakeLists.txt
	echo ')' >> $(WS)/src/ros/cmu/CMakeLists.txt
	$(MAKE) build

test_base:
	$(MAKE) test_base.radlo
	find . -name radlinit.cpp | xargs sed -i'' -e 's/SHMEM.*"/SHMEM0"/'
	$(MAKE) build

use_penn_devices use_cmu_devices use_sri_devices:
	ln -sf devices_$(patsubst use_%_devices,%,$@).radl devices.radl

penn cmu sri:
	$(MAKE) use_$@_devices
	$(MAKE) landshark.radlo
	$(MAKE) ocu.radlo
	$(MAKE) comm.radlo
	$(MAKE) debug.radlo
	$(MAKE) build

penn_plant cmu_plant sri_plant:
	$(MAKE) use_$(patsubst %_plant,%,$@)_devices
	$(MAKE) hacms.radlo
	find . -name radlinit.cpp | xargs sed -i'' -e 's/SHMEM.*"/SHMEM0"/'
	$(MAKE) build


build:
	cd $(WS) &&  catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/hacms/landshark
# -DCMAKE_CXX_FLAGS="-DPUBLISH_SENSOR_MSGS"

$(WS)/src:
	mkdir -p $(WS)/src

etc:
	python scripts/install.py

udev:
	make -C udev

clean:
	-rm -f devices.radl
	-rm -f *.radlo *~
	-rm -rf $(WS)/build
	-rm -rf $(WS)/devel
	-rm -rf $(WS)/install
	-rm -rf udev/build

cleanall: clean
	-rm -rf $(WS)
	-rm -rf *.radlo

uninstall:
	-sudo rm -rf /opt/hacms/*
	-sudo rm -rf /etc/udev/rules.d/9?-*.rules*
	-sudo rm -rf /etc/init.d/landshark

.SECONDEXPANSION:
$(OBJS): $$(subst .radlo,.radl,$$@) $$($$(subst .radlo,,$$@_DEPS)) $(WS)/src
	$(RADLER) --ws_dir $(WS)/src compile $< --ROS -o $@ $(foreach obj,$($(subst .radlo,,$@_DEPS)),-O ${obj}) $($(subst .radlo,,$@_FLAGS))
