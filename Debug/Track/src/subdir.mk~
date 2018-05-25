################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Track/src/VideoProcessTrack.cpp \
../Track/src/mvDetector.cpp 

OBJS += \
./Track/src/VideoProcessTrack.o \
./Track/src/mvDetector.o 

CPP_DEPS += \
./Track/src/VideoProcessTrack.d \
./Track/src/mvDetector.d 


# Each subdirectory must supply rules for building sources it contributes
Track/src/%.o: ../Track/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-8.0/bin/nvcc -DDISABLE_NEON_DEI=1 -DNO_ARM_NEON=1 -DUSE_BMPCAP=1 -DCAM_COUNT=10 -DTRACK_MODE=0 -DUSE_UART=0 -DTEST_GAIN=1 -DUSE_12=1 -DUSE_GAIN=1 -DGSTREAM_CAP=0 -I/usr/include/gstreamer-1.0 -I/usr/include -I../OSA_CAP/inc -I../GLTool/include -I/usr/include/opencv -I/usr/include/GL -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I/usr/include/opencv2 -I../Track/inc -I../Enhance/ -G -g -O0 -ccbin aarch64-linux-gnu-g++ -gencode arch=compute_50,code=sm_50 -m64 -odir "Track/src" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-8.0/bin/nvcc -DDISABLE_NEON_DEI=1 -DNO_ARM_NEON=1 -DUSE_BMPCAP=1 -DCAM_COUNT=10 -DTRACK_MODE=0 -DUSE_UART=0 -DTEST_GAIN=1 -DUSE_12=1 -DUSE_GAIN=1 -DGSTREAM_CAP=0 -I/usr/include/gstreamer-1.0 -I/usr/include -I../OSA_CAP/inc -I/usr/include/opencv -I../GLTool/include -I/usr/include/GL -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I/usr/include/opencv2 -I../Track/inc -I../Enhance/ -G -g -O0 --compile -m64 -ccbin aarch64-linux-gnu-g++  -x c++ -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


