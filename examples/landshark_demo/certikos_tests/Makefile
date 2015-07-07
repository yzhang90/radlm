WS = catkin_ws
RADLER = ../radler/radler.sh

OBJS = test22.radlo test_w_joystick.radlo test_w_joystick_2.radlo

test22_FLAGS = --plant test22
test_w_joystick_FLAGS = --plant test_w_joystick
test_w_joystick_2_FLAGS = --plant test_w_joystick

ssh_alias_SRI_vm = vmsri
ssh_alias_COMM_vm = vmcomm

.PHONY: build all clean cleanall to_lynxsecure

all: $(OBJS)
	$(MAKE) build

to_lynxsecure:
	find . -name \*.radl -print0 | xargs -0 sed -i '' -e 's/certikos/lynxsecure/g'

to_certikos:
	find . -name \*.radl -print0 | xargs -0 sed -i '' -e 's/lynxsecure/certikos/g'


build: $(WS)/src
	find . -name radlinit.cpp -print0 | xargs -0 sed -i '' -e 's/SHMEM.*"/SHMEM0"/'
	cd $(WS) && catkin_make

test22: test22.radlo
	$(MAKE) build
	scp catkin_ws/devel/lib/radlplant_test22/radlinit_6_test22_6_test22_2_bb_2_h1_3_sri_7__linux1 $(ssh_alias_SRI_vm):test22
	scp catkin_ws/devel/lib/radlplant_test22/radlinit_6_test22_6_test22_2_bb_2_h1_4_comm_7__linux1 $(ssh_alias_COMM_vm):test22
	cp catkin_ws/devel/lib/radlplant_test22/radlinit_6_test22_6_test22_3_ocu_7__linux1 test22

stop_test22:
	-ssh $(ssh_alias_SRI_vm) pkill test22
	-ssh $(ssh_alias_COMM_vm) pkill test22
	-pkill test22

test_w_joystick: test_w_joystick.radlo
	$(MAKE) build
	scp catkin_ws/devel/lib/radlplant_test_w_joystick/radlinit_15_test_w_joystick_15_test_w_joystick_9_black_box_10_hypervisor_3_sri_3_sys $(ssh_alias_SRI_vm):test_w_joystick
	scp catkin_ws/devel/lib/radlplant_test_w_joystick/radlinit_15_test_w_joystick_15_test_w_joystick_9_black_box_10_hypervisor_4_comm_3_sys $(ssh_alias_COMM_vm):test_w_joystick
	cp catkin_ws/devel/lib/radlplant_test_w_joystick/radlinit_15_test_w_joystick_15_test_w_joystick_3_ocu_3_sys test_w_joystick

test_w_joystick_2: test_w_joystick_2.radlo
	$(MAKE) build
	scp catkin_ws/devel/lib/radlplant_test_w_joystick/radlinit_17_test_w_joystick_2_15_test_w_joystick_9_black_box_10_hypervisor_3_sri_3_sys $(ssh_alias_SRI_vm):test_w_joystick
	scp catkin_ws/devel/lib/radlplant_test_w_joystick/radlinit_17_test_w_joystick_2_15_test_w_joystick_9_black_box_10_hypervisor_4_comm_3_sys $(ssh_alias_COMM_vm):test_w_joystick

stop_test_w_joystick:
	-ssh $(ssh_alias_SRI_vm) pkill test_w_joystick
	-ssh $(ssh_alias_COMM_vm) pkill test_w_joystick
	-pkill test_w_joystick


$(WS)/src:
	mkdir -p $(WS)/src

clean:
	-rm -f *.radlo *~
	-rm -rf $(WS)/build
	-rm -rf $(WS)/devel
	-rm -rf $(WS)/install

cleanall: clean
	$(MAKE) to_certikos
	-rm -rf $(WS)


# Generate rule for each radl object in OBJ to take car of the dependencies and flags according to the {module name}_DEPS and {module name}_FLAGS variables
.SECONDEXPANSION:
$(OBJS): $$(subst .radlo,.radl,$$@) $$($$(subst .radlo,,$$@_DEPS)) $(WS)/src
	$(RADLER) --ws_dir $(WS)/src compile $< -o $@ $(foreach obj,$($(subst .radlo,,$@_DEPS)),-O ${obj}) $($(subst .radlo,,$@_FLAGS))
