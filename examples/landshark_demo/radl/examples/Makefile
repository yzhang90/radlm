WS = catkin_ws
OBJS = ros.radlo
ros_FLAGS = --plant plant
RADLER = ../../radler/radler.sh

all: examples

examples: ros.radlo
	$(MAKE) build

build: $(WS)
	cd $(WS) && CFLAGS="-std=gnu99 -msse4.2" catkin_make 

$(WS)/src:
	mkdir -p $(WS)/src

# radlo targets
.SECONDEXPANSION:
$(OBJS): $$(subst .radlo,.radl,$$@) $$($$(subst .radlo,,$$@_DEPS)) $(WS)/src
	$(RADLER) --ws_dir $(WS)/src compile $< -o $@ $(foreach obj,$($(subst .radlo,,$@_DEPS)),-O ${obj}) $($(subst .radlo,,$@_FLAGS))

# remove build files
clean:
	-rm -rf $(WS)/build

# remove radl object files, generated files, tar.gz files
cleanall: clean tidy
	-rm -f *.radlo 
	-rm -rf $(WS)

# tidy up workspace 
tidy:
	-rm -rf `find . -iname \*~`

