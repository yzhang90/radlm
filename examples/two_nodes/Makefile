WS = catkin_ws
RADLER = ../../../radler.sh

five_nodes: $(WS)/src
	if [ ! -f five_nodes.radl.bak ]; then cp five_nodes.radl five_nodes.radl.bak; fi
	$(RADLER) monitor_health five_nodes.radl
	$(RADLER) --ws_dir $(WS)/src compile $@.radl --ROS
	make build

build:
	cd $(WS) && catkin_make install

.PHONY: five_nodes build $(WS)/src clean cleanall recovdr

$(WS)/src:
	mkdir -p $(WS)/src

clean:
	-rm -rf $(WS)/build
	-rm -rf $(WS)/devel
	-rm -rf $(WS)/install

cleanall: clean recover
	-rm -rf $(WS)
	-rm -rf `find .. -iname \*.radlo`
	-rm -rf `find -name \*_monitor`

recover:
	cp five_nodes.radl.bak five_nodes.radl
	-rm -rf five_nodes.radl.bak

