WS = catkin_ws
RADLER = ../radler/radler.sh

OBJS = constants.radlo arrays.radlo topics.radlo landshark.radlo ocu.radlo hacms.radlo comm.radlo

topics_DEPS = arrays.radlo
landshark_DEPS = arrays.radlo constants.radlo topics.radlo
ocu_DEPS = arrays.radlo constants.radlo topics.radlo landshark.radlo
hacms_DEPS = arrays.radlo constants.radlo topics.radlo landshark.radlo ocu.radlo comm.radlo
comm_DEPS = arrays.radlo constants.radlo topics.radlo landshark.radlo ocu.radlo

hacms_FLAGS = --plant plant

.PHONY: build sri cmu penn etc clean cleanall uninstall

all: sri

penn cmu sri:
	ln -sf $@_landshark.radl landshark.radl
	$(MAKE) landshark.radlo
	$(MAKE) build

hacms: hacms.radlo


build:
	cd $(WS) && catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/hacms/landshark
# -DCMAKE_CXX_FLAGS="-DPUBLISH_SENSOR_MSGS"

$(WS)/src:
	mkdir -p $(WS)/src

etc:
	python scripts/install.py
	make -C udev

clean:
	-rm -f landshark.radl
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

.SECONDEXPANSION:
$(OBJS): $$(subst .radlo,.radl,$$@) $$($$(subst .radlo,,$$@_DEPS)) $(WS)/src
	$(RADLER) --ws_dir $(WS)/src compile $< -o $@ $(foreach obj,$($(subst .radlo,,$@_DEPS)),-O ${obj}) $($(subst .radlo,,$@_FLAGS))
