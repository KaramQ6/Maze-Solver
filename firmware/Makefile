CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -pedantic -O2 -Iinclude
LDFLAGS ?= -fuse-ld=bfd -lm

SRCS = src/fixed_maze.c src/floodfill.c src/planner.c src/kinematics.c
TEST_SRCS = tests/test_firmware.c

all: test

test: build/test_firmware
	./build/test_firmware

build/test_firmware: $(SRCS) $(TEST_SRCS)
	mkdir -p build
	$(CC) $(CFLAGS) $(SRCS) $(TEST_SRCS) -o $@ $(LDFLAGS)

clean:
	rm -rf build

.PHONY: all test clean
