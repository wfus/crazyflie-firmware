AS      = $(GCC_BIN)arm-none-eabi-as
CC      = $(GCC_BIN)arm-none-eabi-gcc
CXX    = $(GCC_BIN)arm-none-eabi-g++
LD      = $(GCC_BIN)arm-none-eabi-gcc
OBJCOPY = $(GCC_BIN)arm-none-eabi-objcopy
OBJDUMP = $(GCC_BIN)arm-none-eabi-objdump
SIZE    = $(GCC_BIN)arm-none-eabi-size 

SRCS := \
tensorflow/lite/experimental/micro/micro_error_reporter.cc \
tensorflow/lite/experimental/micro/micro_mutable_op_resolver.cc \
tensorflow/lite/experimental/micro/simple_tensor_allocator.cc \
tensorflow/lite/experimental/micro/debug_log.cc \
tensorflow/lite/experimental/micro/debug_log_numbers.cc \
tensorflow/lite/experimental/micro/micro_interpreter.cc \
tensorflow/lite/experimental/micro/kernels/depthwise_conv.cc \
tensorflow/lite/experimental/micro/kernels/softmax.cc \
tensorflow/lite/experimental/micro/kernels/all_ops_resolver.cc \
tensorflow/lite/experimental/micro/kernels/fully_connected.cc \
tensorflow/lite/c/c_api_internal.c \
tensorflow/lite/core/api/error_reporter.cc \
tensorflow/lite/core/api/flatbuffer_conversions.cc \
tensorflow/lite/core/api/op_resolver.cc \
tensorflow/lite/kernels/kernel_util.cc \
tensorflow/lite/kernels/internal/quantization_util.cc \
tensorflow/lite/experimental/micro/examples/micro_speech/model_settings.cc \
tensorflow/lite/experimental/micro/examples/micro_speech/audio_provider.cc \
tensorflow/lite/experimental/micro/examples/micro_speech/feature_provider.cc \
tensorflow/lite/experimental/micro/examples/micro_speech/preprocessor.cc \
tensorflow/lite/experimental/micro/examples/micro_speech/no_features_data.cc \
tensorflow/lite/experimental/micro/examples/micro_speech/yes_features_data.cc \
tensorflow/lite/experimental/micro/examples/micro_speech/tiny_conv_model_data.cc \
tensorflow/lite/experimental/micro/examples/micro_speech/recognize_commands.cc \
main.c \


OBJS := \
$(patsubst %.cc,%.o,$(patsubst %.c,%.o,$(SRCS)))

INCLUDES := \
-I. \
-I./third_party/gemmlowp \
-I./third_party/flatbuffers/include

CCFLAGS += --specs=nosys.specs -fpermissive
CXXFLAGS += --specs=nosys.specs -Os -fpermissive -DNDEBUG --std=c++11 -g -DTF_LITE_STATIC_MEMORY


# CC and CXX for testing purposes (so you can compile it on your laptop/desktop)
# We should still be able to link our C main function to our C++ methods in
# TF Micro.
TEST_CC = gcc
TEST_CXX = g++
TEST_CCFLAGS = -Os
TEST_CXXFLAGS = -Os -std=gnu++11

ifeq ($(DEBUG), 1)
	COMPILE_CC = $(TEST_CC)
	COMPILE_CXX = $(TEST_CXX)
	COMPILE_CCFLAGS = $(TEST_CCFLAGS)
	COMPILE_CXXFLAGS = $(TEST_CXXFLAGS)
else
	COMPILE_CC = $(CC)
	COMPILE_CXX = $(CXX)
	COMPILE_CCFLAGS = $(CCFLAGS)
	COMPILE_CXXFLAGS = $(CXXFLAGS)
endif

LDFLAGS += -lm --specs=nosys.specs

# Use the DEBUG flag to compile on x86 for testing purposes to see if
# linking works on a more accessible device.
%.o: %.cc
	$(COMPILE_CXX) $(COMPILE_CXXFLAGS) $(INCLUDES) -c $< -o $@

%.o: %.c
	$(COMPILE_CC) $(COMPILE_CCFLAGS) $(INCLUDES) -c $< -o $@

tfmicrotest : $(OBJS)
	$(CXX) $(LDFLAGS) $(OBJS) \
	-o $@


all: tfmicrotest 


clean :
	$(RM) $(OBJS) 

