all:
	mkdir -p build/
	cd build/ && cmake .. -DCMAKE_BUILD_TYPE=Release
	$(MAKE) -C build

debug:
	mkdir -p build/
	cd build/ && cmake .. -DCMAKE_BUILD_TYPE=Debug
	$(MAKE) -C build

test:
	mkdir -p out/
	$(MAKE) -C build test

clean:
	rm -rf build/ ./CMakeFiles

.PHONY: all clean test
