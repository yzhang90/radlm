# Makefile for upenn only 
#
WS = catkin_ws
RADLER = ../../radler/radler.sh

OBJS = constants.radlo arrays.radlo topics.radlo global.radlo penn.radlo

topics_DEPS = global.radlo arrays.radlo
penn_DEPS = global.radlo arrays.radlo constants.radlo topics.radlo 

.PHONY: build sri cmu penn etc clean cleanall uninstall

all: penn.radlo
	$(MAKE) build

build:
	cd $(WS) && catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/hacms/landshark

$(WS)/src:
	mkdir -p $(WS)/src

clean:
	-rm -f *.radlo *~
	-rm -rf $(WS)/build
	-rm -rf $(WS)/devel
	-rm -rf $(WS)/install

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
