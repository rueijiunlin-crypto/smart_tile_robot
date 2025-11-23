# Tool macros
CC ?= gcc
CXX ?= g++

# Settings
BUILD_PATH = ./build

# SDK 路徑
SDK_DIR = sdk

# Search path for header files
CFLAGS += -I.
CFLAGS += -I$(SDK_DIR)

# C and C++ Compiler flags
CFLAGS += -Wall						# Include all warnings
CFLAGS += -g						# Generate GDB debugger information
CFLAGS += -Wno-strict-aliasing		# Disable warnings about strict aliasing
CFLAGS += -Os						# Optimize for size
CFLAGS += -DNDEBUG					# Disable assert() macro

# C++ only compiler flags
CXXFLAGS += -std=c++11				# Use C++11 standard

# Linker flags
LDFLAGS += -lm 						# Link to math.h
LDFLAGS += -lstdc++					# Link to stdc++.h

# Include C source code for required libraries
CSOURCES += $(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/DSP/Source/TransformFunctions/*.c) \
			$(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/DSP/Source/CommonTables/*.c) \
			$(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/DSP/Source/BasicMathFunctions/*.c) \
			$(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/DSP/Source/ComplexMathFunctions/*.c) \
			$(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/DSP/Source/FastMathFunctions/*.c) \
			$(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/DSP/Source/SupportFunctions/*.c) \
			$(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/DSP/Source/MatrixFunctions/*.c) \
			$(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/DSP/Source/StatisticsFunctions/*.c)

# Include C++ source code for required libraries (共用部分)
CXXSOURCES_COMMON = 	$(wildcard $(SDK_DIR)/tflite-model/*.cpp) \
						$(wildcard $(SDK_DIR)/edge-impulse-sdk/dsp/kissfft/*.cpp) \
						$(wildcard $(SDK_DIR)/edge-impulse-sdk/dsp/dct/*.cpp) \
						$(wildcard $(SDK_DIR)/edge-impulse-sdk/dsp/memory.cpp) \
						$(wildcard $(SDK_DIR)/edge-impulse-sdk/porting/posix/*.c*) \
						$(wildcard $(SDK_DIR)/edge-impulse-sdk/porting/mingw32/*.c*)

CCSOURCES +=

# Use TensorFlow Lite for Microcontrollers (TFLM)
CFLAGS += -DTF_LITE_DISABLE_X86_NEON=1
CSOURCES +=	$(SDK_DIR)/edge-impulse-sdk/tensorflow/lite/c/common.c
CCSOURCES +=	$(wildcard $(SDK_DIR)/edge-impulse-sdk/tensorflow/lite/kernels/*.cc) \
				$(wildcard $(SDK_DIR)/edge-impulse-sdk/tensorflow/lite/kernels/internal/*.cc) \
				$(wildcard $(SDK_DIR)/edge-impulse-sdk/tensorflow/lite/micro/kernels/*.cc) \
				$(wildcard $(SDK_DIR)/edge-impulse-sdk/tensorflow/lite/micro/*.cc) \
				$(wildcard $(SDK_DIR)/edge-impulse-sdk/tensorflow/lite/micro/memory_planner/*.cc) \
				$(wildcard $(SDK_DIR)/edge-impulse-sdk/tensorflow/lite/core/api/*.cc)

# Include CMSIS-NN if compiling for an Arm target that supports it
ifeq (${CMSIS_NN}, 1)

	# Include CMSIS-NN and CMSIS-DSP header files
	CFLAGS += -I$(SDK_DIR)/edge-impulse-sdk/CMSIS/NN/Include/
	CFLAGS += -I$(SDK_DIR)/edge-impulse-sdk/CMSIS/DSP/PrivateInclude/

	# C and C++ compiler flags for CMSIS-NN and CMSIS-DSP
	CFLAGS += -Wno-unknown-attributes 					# Disable warnings about unknown attributes
	CFLAGS += -DEI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN=1	# Use CMSIS-NN functions in the SDK
	CFLAGS += -D__ARM_FEATURE_DSP=1 					# Enable CMSIS-DSP optimized features
	CFLAGS += -D__GNUC_PYTHON__=1						# Enable CMSIS-DSP intrisics (non-C features)

	# Include C source code for required CMSIS libraries
	CSOURCES += $(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/NN/Source/ActivationFunctions/*.c) \
				$(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/NN/Source/BasicMathFunctions/*.c) \
				$(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/NN/Source/ConcatenationFunctions/*.c) \
				$(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/NN/Source/ConvolutionFunctions/*.c) \
				$(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/NN/Source/FullyConnectedFunctions/*.c) \
				$(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/NN/Source/NNSupportFunctions/*.c) \
				$(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/NN/Source/PoolingFunctions/*.c) \
				$(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/NN/Source/ReshapeFunctions/*.c) \
				$(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/NN/Source/SoftmaxFunctions/*.c) \
				$(wildcard $(SDK_DIR)/edge-impulse-sdk/CMSIS/NN/Source/SVDFunctions/*.c)
endif

# Generate names for the output object files (*.o)
COBJECTS := $(patsubst %.c,%.o,$(CSOURCES))
CCOBJECTS := $(patsubst %.cc,%.o,$(CCSOURCES))

# 兩個主程式的源檔案
CXXSOURCES_BATCH = source/main_batch.cpp $(CXXSOURCES_COMMON)
CXXSOURCES_LATEST = source/main_latest.cpp $(CXXSOURCES_COMMON)

# 對應的目標檔案
CXXOBJECTS_BATCH := $(patsubst %.cpp,%.o,$(CXXSOURCES_BATCH))
CXXOBJECTS_LATEST := $(patsubst %.cpp,%.o,$(CXXSOURCES_LATEST))

# Default rule
.PHONY: all
all: batch latest

# Compile library source code into object files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.cc
	$(CXX) $(CFLAGS) $(CXXFLAGS) -c $< -o $@

%.o: %.cpp
	$(CXX) $(CFLAGS) $(CXXFLAGS) -c $< -o $@

# Build batch inference program
.PHONY: batch
batch: $(COBJECTS) $(CCOBJECTS) $(CXXOBJECTS_BATCH)
ifeq ($(OS), Windows_NT)
	if not exist build mkdir build
else
	mkdir -p $(BUILD_PATH)
endif
	$(CXX) $(COBJECTS) $(CCOBJECTS) $(CXXOBJECTS_BATCH) -o $(BUILD_PATH)/inference_batch $(LDFLAGS)

# Build latest folder inference program
.PHONY: latest
latest: $(COBJECTS) $(CCOBJECTS) $(CXXOBJECTS_LATEST)
ifeq ($(OS), Windows_NT)
	if not exist build mkdir build
else
	mkdir -p $(BUILD_PATH)
endif
	$(CXX) $(COBJECTS) $(CCOBJECTS) $(CXXOBJECTS_LATEST) -o $(BUILD_PATH)/inference_latest $(LDFLAGS)

# Remove compiled object files
.PHONY: clean
clean:
ifeq ($(OS), Windows_NT)
	del /Q $(subst /,\,$(patsubst %.c,%.o,$(CSOURCES))) >nul 2>&1 || exit 0
	del /Q $(subst /,\,$(patsubst %.cpp,%.o,$(CXXSOURCES_BATCH))) >nul 2>&1 || exit 0
	del /Q $(subst /,\,$(patsubst %.cpp,%.o,$(CXXSOURCES_LATEST))) >nul 2>&1 || exit 0
	del /Q $(subst /,\,$(patsubst %.cc,%.o,$(CCSOURCES))) >nul 2>&1 || exit 0
else
	rm -f $(COBJECTS)
	rm -f $(CCOBJECTS)
	rm -f $(CXXOBJECTS_BATCH)
	rm -f $(CXXOBJECTS_LATEST)
	rm -rf $(BUILD_PATH)
endif

