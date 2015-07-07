all:
	@mkdir -p build
	cd build && cmake $(CMAKE_FLAGS) -DCMAKE_INSTALL_PREFIX=/opt/hacms .. 
	cd build && make install

clean:
	-cd build && make clean
	rm -rf build

