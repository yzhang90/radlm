RADLM = ../radlm.sh

.PHONY: two_nodes

all:

two_nodes:
	$(RADLM) gen $@/ $@/monitor.spec
	$(RADLM) trans $@_gen/ $@_gen/monitor.radlm

clean:
	-rm -rf `find . -iname \*_gen`
	-rm -rf `find . -iname \*_trans`
