WS = catkin_ws
RADLER = ../../radler/radler.sh

OBJS = two_nodes.radlo monitor.radlo monitor_topics.radlo

two_nodes_DEPS = monitor_topics.radlo
monitor_DEPS = monitor_topics.radlo two_nodes.radlo

.PHONY: two_nodes build clean

two_nodes: $(WS)/src
	$(MAKE) monitor.radlo
	cd $(WS) && catkin_make install

$(WS)/src:
	mkdir -p $(WS)/src

clean:
	-rm -rf $(WS)

.SECONDEXPANSION:
$(OBJS): $$(subst .radlo,.radl,$$@) $$($$(subst .radlo,,$$@_DEPS)) $(WS)/src
	$(RADLER) --ws_dir $(WS)/src compile $< -o $@ $(foreach obj,$($(subst .radlo,,$@_DEPS)),-O ${obj}) --ROS
