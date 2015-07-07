WS = catkin_ws
INSTALL=opt/hacms/landshark
RADLER = ../radler/radler.sh

# tar.gz for each VM on lynxsecure
LS_TGZS = tgz/comm.tar.gz tgz/sri.tar.gz tgz/upenn.tar.gz tgz/cmu.tar.gz tgz/ocu.tar.gz 

# tar.gz for each VM on lynxsecure
BAREMETAL_TGZS = tgz/bm_ocu.tar.gz tgz/bm_blackbox.tar.gz 

# tar.gz for each VM on Certikos
CERTIKOS_TGZS = tgz/vm1.tar.gz tgz/vm2.tar.gz tgz/vm3.tar.gz tgz/vm4.tar.gz tgz/vm5.tar.gz tgz/ocu.tar.gz 

# tar.gz for kestrel back-end
KESTREL_TGZS = tgz/vm1.tar.gz tgz/vm2.tar.gz tgz/vm3.tar.gz tgz/vm4.tar.gz tgz/ocu.tar.gz 

.PHONY: build sri_plant cmu_plant penn_plant clean cleanall uninstall udev cmu_ros ocu_gui certikos_tgz $(KESTREL_TGZS)

# RADL object files
OBJS = constants.radlo arrays.radlo topics.radlo landshark.radlo ocu.radlo comm.radlo global.radlo devices.radlo cmu/cmu.radlo upenn/penn.radlo certikos.radlo baremetal.radlo core.radlo lynxsecure.radlo mtopics.radlo demo.radlo

# RADL dependencies 
topics_DEPS = global.radlo arrays.radlo
mtopics_DEPS = global.radlo arrays.radlo
landshark_DEPS = global.radlo arrays.radlo constants.radlo topics.radlo mtopics.radlo devices.radlo
ocu_DEPS = global.radlo arrays.radlo constants.radlo topics.radlo mtopics.radlo 
comm_DEPS = global.radlo arrays.radlo constants.radlo topics.radlo mtopics.radlo ocu.radlo

demo_DEPS = global.radlo arrays.radlo topics.radlo mtopics.radlo

cmu/cmu_DEPS = global.radlo arrays.radlo constants.radlo topics.radlo 
upenn/penn_DEPS = global.radlo arrays.radlo constants.radlo topics.radlo

lynxsecure_DEPS = global.radlo arrays.radlo constants.radlo topics.radlo mtopics.radlo devices.radlo landshark.radlo ocu.radlo comm.radlo upenn/penn.radlo cmu/cmu.radlo demo.radlo 
certikos_DEPS = global.radlo arrays.radlo constants.radlo topics.radlo mtopics.radlo devices.radlo landshark.radlo ocu.radlo comm.radlo upenn/penn.radlo cmu/cmu.radlo demo.radlo
baremetal_DEPS = global.radlo arrays.radlo constants.radlo topics.radlo mtopics.radlo devices.radlo landshark.radlo ocu.radlo comm.radlo upenn/penn.radlo cmu/cmu.radlo demo.radlo

core_DEPS = global.radlo arrays.radlo constants.radlo topics.radlo 

# RADL plant flags
lynxsecure_FLAGS = --ROSplant --plant plant
certikos_FLAGS = --plant plant
baremetal_FLAGS = --plant plant
core_FLAGS = --plant plant

all: sri_baremetal_plant 

use_penn_devices use_cmu_devices use_sri_devices:
	ln -sf devices_$(patsubst use_%_devices,%,$@).radl devices.radl

penn_ls_plant cmu_ls_plant sri_ls_plant:
	@echo "Building $@"
	$(MAKE) use_$(patsubst %_ls_plant,%,$@)_devices
	# To temporarily fix lynxsecure generated code.
	#find . -name radlinit.cpp | xargs sed -i'' -e 's/SHMEM.*"/SHMEM0"/'
	$(MAKE) lynxsecure.radlo
	$(MAKE) build

penn_baremetal_plant cmu_baremetal_plant sri_baremetal_plant:
	@echo "Building $@"
	$(MAKE) use_$(patsubst %_baremetal_plant,%,$@)_devices
	$(MAKE) baremetal.radlo
	$(MAKE) build

penn_certikos_plant cmu_certikos_plant sri_certikos_plant:
	@echo "Building $@"
	$(MAKE) use_$(patsubst %_certikos_plant,%,$@)_devices
	$(MAKE) certikos.radlo
	$(MAKE) build

core_plant:
	@echo "Building $@"
	$(MAKE) core.radlo
	$(MAKE) build

build: $(WS)
	mkdir -p ${WS}/${INSTALL}
	cd $(WS) && CFLAGS="-std=gnu99 -msse4.2" catkin_make install -DCMAKE_INSTALL_PREFIX=${INSTALL} 

$(WS)/src:
	mkdir -p $(WS)/src

# baremetal install
core_tgz: udev $(CORE_TGZS)

# baremetal install
baremetal_tgz: udev ocu_gui cmu_ros $(BAREMETAL_TGZS)
#baremetal_tgz: udev ocu_gui $(BAREMETAL_TGZS)

baremetal_install: baremetal_tgz
	sudo tar xzf tgz/bm_blackbox.tar.gz -C /
	scp tgz/bm_ocu.tar.gz ocu:
	@echo ""
	@echo "To finish installing on OCU, do: "
	@echo "    ssh ocu"
	@echo "    sudo tar xzf bm_ocu.tar.gz -C /"


# certikos install
certikos_tgz: udev ocu_gui $(CERTIKOS_TGZS)

certikos_install: certikos_tgz
	sudo tar xzf tgz/vm1.tar.gz -C /media/${USER}/vm1
	sudo tar xzf tgz/vm2.tar.gz -C /media/${USER}/vm2
	sudo tar xzf tgz/vm3.tar.gz -C /media/${USER}/vm3
	sudo tar xzf tgz/vm4.tar.gz -C /media/${USER}/vm4
	sudo tar xzf tgz/vm5.tar.gz -C /media/${USER}/vm5
	scp tgz/ocu.tar.gz ocu:
	@echo ""
	@echo "To finish installing on OCU, do: "
	@echo "    ssh ocu"
	@echo "    sudo tar xzf ocu.tar.gz -C /"

# lynxsecure install
ls_tgz: udev ocu_gui $(LS_TGZS)

ls_install:
	scp files/Makefile tgz/comm.tar.gz comm:
	scp files/Makefile tgz/sri.tar.gz sri:
	scp files/Makefile tgz/upenn.tar.gz upenn:
	scp files/Makefile tgz/cmu.tar.gz cmu:
	ssh comm "tar xzf comm.tar.gz -C / --exclude=etc"
	ssh sri "tar xzf sri.tar.gz -C / --exclude=etc"
	ssh cmu "tar xzf cmu.tar.gz -C / --exclude=etc"
	ssh upenn "tar xzf upenn.tar.gz -C / --exclude=etc"

# Non-radl targets 
udev:
	@mkdir -p udev/build
	cd udev/build && cmake $(CMAKE_FLAGS) .. -DCMAKE_INSTALL_PREFIX=../../generated/all/opt/hacms && make install

ocu_gui:
	@mkdir -p $@/build
	cd $@/build && cmake $(CMAKE_FLAGS) .. -DCMAKE_INSTALL_PREFIX=../../generated/ocu/opt/hacms .. && make install

cmu_ros:
	cd cmu_ros && catkin_make install -DCMAKE_INSTALL_PREFIX=../generated/cmu/opt/hacms/cmu

# tidy up workspace 
tidy:
	-rm -rf `find . -iname \*~`

# remove build files
clean:
	-rm -rf $(WS)/build
	-rm -rf udev/build
	-rm -rf cmu_ros/build
	-rm -rf ocu_gui/build

# remove radl object files, generated files, tar.gz files
cleanall: clean tidy
	-rm -f devices.radl
	-rm -f *.radlo */*.radlo 
	-rm -rf $(WS)
	-rm -rf tgz
	-rm -rf generated

# remove installed files inside VMs
uninstall:
	-sudo rm -rf /opt/hacms/*
	-sudo rm -rf /etc/udev/rules.d/[679]?-*.rules*
	-sudo rm -f /etc/init.d/landshark
	-sudo rm -f /etc/init.d/plant

# radlo targets
.SECONDEXPANSION:
$(OBJS): $$(subst .radlo,.radl,$$@) $$($$(subst .radlo,,$$@_DEPS)) $(WS)/src
	$(RADLER) --ws_dir $(WS)/src compile $< -o $@ $(foreach obj,$($(subst .radlo,,$@_DEPS)),-O ${obj}) $($(subst .radlo,,$@_FLAGS))

# certikos tar.gz
.SECONDEXPANSION:
$(KESTREL_TGZS): 
	-rm -f $@ $(subst .gz,,$@)
	mkdir -p `dirname $@`
	chmod 0600 files/kestrel/*/etc/racoon/certs/*
	mkdir -p generated/$(subst .tar.gz,,$(subst tgz/,,$@))/opt/hacms/bin
	cp $(WS)/devel/lib/radlplant_plant/radlinit*$(subst .tar.gz,,$(subst tgz/,,$@))* generated/$(subst .tar.gz,,$(subst tgz/,,$@))/opt/hacms/bin/radlinit
	tar rf $(subst .gz,,$@) -C generated/all `ls generated/all` 
	tar rf $(subst .gz,,$@) -C generated/$(subst .tar.gz,,$(subst tgz/,,$@)) `ls generated/$(subst .tar.gz,,$(subst tgz/,,$@))` 
	tar rf $(subst .gz,,$@) -C files/kestrel/all `ls files/kestrel/all` 
	tar rf $(subst .gz,,$@) -C files/kestrel/$(subst .tar.gz,,$(subst tgz/,,$@)) `ls files/kestrel/$(subst .tar.gz,,$(subst tgz/,,$@))` #--exclude=iptablesload
	gzip $(subst .gz,,$@)

# baremetal blackbox without firewall
.SECONDEXPANSION:
tgz/bm_blackbox.tar.gz: 
	-rm -f $@ $(subst .gz,,$@)
	mkdir -p `dirname $@`
	mkdir -p generated/$(subst .tar.gz,,$(subst tgz/bm_,,$@))/opt/hacms/bin
	cp $(WS)/devel/lib/radlplant_plant/radlinit*$(subst .tar.gz,,$(subst tgz/bm_,,$@))* generated/$(subst .tar.gz,,$(subst tgz/bm_,,$@))/opt/hacms/bin/radlinit
	tar rf $(subst .gz,,$@) -C generated/all `ls generated/all` 
	tar rf $(subst .gz,,$@) -C generated/cmu `ls generated/cmu` 
	tar rf $(subst .gz,,$@) -C generated/$(subst .tar.gz,,$(subst tgz/bm_,,$@)) `ls generated/$(subst .tar.gz,,$(subst tgz/bm_,,$@))` 
	tar rf $(subst .gz,,$@) -C files/kestrel/all `ls files/kestrel/all`
	tar rf $(subst .gz,,$@) -C files/kestrel/cmu `ls files/kestrel/cmu` 
	tar rf $(subst .gz,,$@) -C files/kestrel/$(subst .tar.gz,,$(subst tgz/bm_,,$@)) `ls files/kestrel/$(subst .tar.gz,,$(subst tgz/bm_,,$@))` --exclude=iptablesload
	gzip $(subst .gz,,$@)
	@echo "Warning!!! Excluding firewall: iptablesload"

# baremetal ocu without firewall
tgz/bm_ocu.tar.gz: 
	-rm -f $@ $(subst .gz,,$@)
	mkdir -p `dirname $@`
	mkdir -p generated/$(subst .tar.gz,,$(subst tgz/bm_,,$@))/opt/hacms/bin
	cp $(WS)/devel/lib/radlplant_plant/radlinit*$(subst .tar.gz,,$(subst tgz/bm_,,$@))* generated/$(subst .tar.gz,,$(subst tgz/bm_,,$@))/opt/hacms/bin/radlinit
	tar rf $(subst .gz,,$@) -C generated/all `ls generated/all` 
	tar rf $(subst .gz,,$@) -C generated/$(subst .tar.gz,,$(subst tgz/bm_,,$@)) `ls generated/$(subst .tar.gz,,$(subst tgz/bm_,,$@))` 
	tar rf $(subst .gz,,$@) -C files/kestrel/all `ls files/kestrel/all`
	tar rf $(subst .gz,,$@) -C files/kestrel/$(subst .tar.gz,,$(subst tgz/bm_,,$@)) `ls files/kestrel/$(subst .tar.gz,,$(subst tgz/bm_,,$@))` --exclude=iptablesload
	gzip $(subst .gz,,$@)

# baremetal core install 
tgz/core.tar.gz:
	-rm -f $@ $(subst .gz,,$@)
	mkdir -p `dirname $@`
	mkdir -p generated/core/opt/hacms/bin
	cp $(WS)/devel/lib/radlplant_plant/radlinit*core* generated/core/opt/hacms/bin/radlinit
	tar rf $(subst .gz,,$@) -C generated/core `ls generated/core` 
	tar rf $(subst .gz,,$@) -C files/kestrel/all `ls files/kestrel/all`
	tar rf $(subst .gz,,$@) -C files/kestrel/blackbox `ls files/kestrel/blackbox` --exclude=iptablesload
	gzip $(subst .gz,,$@)

tgz/vm5.tar.gz tgz/red.tar.gz::
	-rm -f $@ $(subst .gz,,$@)
	mkdir -p `dirname $@`
	tar rf $(subst .gz,,$@) -C files/kestrel/$(subst .tar.gz,,$(subst tgz/,,$@)) `ls files/kestrel/$(subst .tar.gz,,$(subst tgz/,,$@))` 
	gzip $(subst .gz,,$@)

