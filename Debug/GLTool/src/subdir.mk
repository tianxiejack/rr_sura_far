################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../GLTool/src/GLBatch.cpp \
../GLTool/src/GLShaderManager.cpp \
../GLTool/src/GLTools.cpp \
../GLTool/src/GLTriangleBatch.cpp \
../GLTool/src/math3d.cpp 

OBJS += \
./GLTool/src/GLBatch.o \
./GLTool/src/GLShaderManager.o \
./GLTool/src/GLTools.o \
./GLTool/src/GLTriangleBatch.o \
./GLTool/src/math3d.o 

CPP_DEPS += \
./GLTool/src/GLBatch.d \
./GLTool/src/GLShaderManager.d \
./GLTool/src/GLTools.d \
./GLTool/src/GLTriangleBatch.d \
./GLTool/src/math3d.d 


# Each subdirectory must supply rules for building sources it contributes
GLTool/src/%.o: ../GLTool/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-8.0/bin/nvcc -DDISABLE_NEON_DEI=1 -DNO_ARM_NEON=1 -DUSE_BMPCAP=1 -DCAM_COUNT=10 -DTRACK_MODE=0 -DUSE_UART=0 -DTEST_GAIN=1 -DUSE_12=1 -DUSE_GAIN=1 -DGSTREAM_CAP=0 -I/usr/include/gstreamer-1.0 -I/usr/include -I../OSA_CAP/inc -I../GLTool/include -I/usr/include/GL -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I/usr/include/opencv2 -I/usr/include/opencv -I../Enhance/ -G -g -O0 -ccbin aarch64-linux-gnu-g++ -gencode arch=compute_50,code=sm_50 -m64 -odir "GLTool/src" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-8.0/bin/nvcc -DDISABLE_NEON_DEI=1 -DNO_ARM_NEON=1 -DUSE_BMPCAP=1 -DCAM_COUNT=10 -DTRACK_MODE=0 -DUSE_UART=0 -DTEST_GAIN=1 -DUSE_12=1 -DUSE_GAIN=1 -DGSTREAM_CAP=0 -I/usr/include/gstreamer-1.0 -I/usr/include -I../OSA_CAP/inc -I../GLTool/include -I/usr/include/GL -I/usr/include/opencv -I/usr/include/opencv2 -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I../Enhance/ -G -g -O0 --compile -m64 -ccbin aarch64-linux-gnu-g++  -x c++ -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


