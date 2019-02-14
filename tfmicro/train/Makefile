TF_BASE_DIR = /Users/wfu/tensorflow

CXX = g++
CXXFLAGS += --std=gnu++11

INCLUDES = \
-I$(TF_BASE_DIR)tensorflow/lite \
-I$(TF_BASE_DIR) \
-I. \
-I$(TF_BASE_DIR)/flatbuffers/include
LDFLAGS = $(TF_BASE_DIR)/bazel-bin/tensorflow/lite
LIBS = tensorflowlite

test_mnist: test.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) test.cpp -L$(LDFLAGS) -l$(LIBS) -o test

all: test_mnist
