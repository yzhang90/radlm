
.phony: all test clean

all: pervasives/radlast_4_radl pervasives/radl.radlo pervasives/ros

pervasives/radlast_4_radl: test
	echo "4"
pervasives/radl.radlo: test
	echo "o"
pervasives/ros: test
	echo "s"

test:
	@echo "  Doing a dry compilation to setup radler internals."
	D="`mktemp -d`" && touch "$$D/t.radl" && ./radler.sh --ws_dir "$$D" compile "$$D/t.radl" && $(RM) -r $$D

clean:
	$(RM) -r pervasives/radlast_4_radl pervasives/radl.radlo pervasives/ros
