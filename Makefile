all: build
	cd build && cmake .. && make

build:
	mkdir build

clean:
	rm -rf ./build
