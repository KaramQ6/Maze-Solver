CC ?= gcc
CXX ?= g++
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -pedantic -O2 -Iinclude
LDFLAGS ?= -fuse-ld=bfd -lm

SRCS = src/fixed_maze.c src/floodfill.c src/planner.c src/kinematics.c
TEST_SRCS = tests/test_firmware.c

all: test

test-c3: build/test_c3_motion build/test_c3_sensors
	./build/test_c3_motion
	./build/test_c3_sensors

build/test_c3_motion: src/motors.cpp src/controller.cpp test/test_c3_motion.cpp test/host/Arduino.h include/config.h
	mkdir -p build
	$(CXX) -std=c++11 -Wall -Wextra -Werror -DARDUINO=10819 -DROBOT_ESP32_C3=1 -Itest/host -Iinclude src/motors.cpp src/controller.cpp test/test_c3_motion.cpp -o $@ $(LDFLAGS)

build/test_c3_sensors: src/sensors.cpp test/test_c3_sensors.cpp test/host/Arduino.h test/host/Wire.h test/host/VL53L0X.h include/config.h
	mkdir -p build
	$(CXX) -std=c++11 -Wall -Wextra -Werror -DARDUINO=10819 -DROBOT_ESP32_C3=1 -Itest/host -Iinclude src/sensors.cpp test/test_c3_sensors.cpp -o $@ $(LDFLAGS)

test: build/test_firmware
	./build/test_firmware

build/test_firmware: $(SRCS) $(TEST_SRCS)
	mkdir -p build
	$(CC) $(CFLAGS) $(SRCS) $(TEST_SRCS) -o $@ $(LDFLAGS)

clean:
	rm -rf build

.PHONY: all test test-c3 clean
