# Default target
all: build

# Create build directory and run cmake + make
build:
	mkdir -p build
	cd build && cmake ..
	cd build && make
	mv build/astars ./astars 

# Clean build directory
clean:
	rm -rf build

# Declare phony targets
.PHONY: all build clean