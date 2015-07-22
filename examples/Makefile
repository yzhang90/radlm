RADLM = ../radlm.sh

.PHONY: two_nodes_plant two_nodes

two_nodes_plant:
	 $(RADLM) weave $@/ $@/monitor.radlm

clean:
	-rm -rf `find . -iname \*_trans`
