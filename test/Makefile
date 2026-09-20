# Host build for the unit tests. Needs nothing but a C++ compiler.
#
#   make            build and run
#   make build/run_tests   build only
#   make clean

CXX      ?= g++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -Wpedantic -Wshadow -Wconversion \
            -Wsign-conversion -O1 -g
BUILD    := build
BIN      := $(BUILD)/run_tests

CORE_SRCS := $(wildcard ../src/*.cpp)
TEST_SRCS := $(wildcard *.cpp)
SRCS      := $(CORE_SRCS) $(TEST_SRCS)
HDRS      := $(wildcard ../src/*.h) $(wildcard *.h)

.PHONY: all test clean
all: test

$(BIN): $(SRCS) $(HDRS)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(BIN)

test: $(BIN)
	@./$(BIN)

clean:
	@rm -rf $(BUILD)
