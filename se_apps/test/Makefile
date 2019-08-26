.PHONY: all
all:
	mkdir -p build/
	cd build/ && cmake .. -DCMAKE_BUILD_TYPE=Release
	$(MAKE) -C build

.PHONY: debug
debug:
	mkdir -p build/
	cd build/ && cmake .. -DCMAKE_BUILD_TYPE=Debug
	$(MAKE) -C build

.PHONY: test
test:
	$(MAKE) -C build test

.PHONY: clean
clean:
	rm -rf build/ ./CMakeFiles

