################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ClicktoMoveForesight.cpp\
../spiH.cpp\
../Cap_Spi_Message.cpp\
../MvDetect.cpp\
../thread_idle.cpp\
../Thread_Priority.cpp\
../MiscCaptureGroup.cpp\
../BMPCaptureGroup.cpp\
../ChosenCaptureGroup.cpp\
../PanoCaptureGroup.cpp\
../GLEnv.cpp\
../doubleScreen.cpp\
../AlarmTarget.cpp \
../Camera.cpp \
../CaptureGroup.cpp \
../CheckMyself.cpp \
../CornerMarker.cpp \
../DataofAlarmarea.cpp \
../DynamicTrack.cpp \
../ExposureCompensationThread.cpp \
../FBOManager.cpp \
../FishCalib.cpp \
../ForeSight.cpp \
../GLRender.cpp \
../HDV4lcap.cpp \
../LineofRuler.cpp \
../Loader.cpp \
../OitVehicle.cpp \
../PBOManager.cpp \
../PBO_FBO_Facade.cpp \
../Parayml.cpp \
../PresetCameraGroup.cpp \
../ProcessIPCMsg.cpp \
../RenderDrawBehvrImpl.cpp \
../RenderMain.cpp \
../STLASCIILoader.cpp \
../SelfCheckThread.cpp \
../Serial_port.cpp \
../ShaderParamArrays.cpp \
../Zodiac_GPIO_Message.cpp \
../Zodiac_Message.cpp \
../buffer.cpp \
../common.cpp \
../deinterlacing.cpp \
../glInfo.cpp \
../glm.cpp \
../gst_capture.cpp \
../main.cpp \
../overLap.cpp \
../overLapBuffer.cpp \
../overLapRegion.cpp \
../pboProcessSrcThread.cpp \
../recvUARTdata.cpp \
../recvwheeldata.cpp \
../scanner.cpp \
../thread.cpp \
../updatebuffer.cpp \
../Xin_IPC_Yuan_Recv_Message.cpp \
../set_button.cpp \
../v4l2camera.cpp 

OBJ_SRCS += \
../tank1215_b_m1.obj 

OBJS += \
./ClicktoMoveForesight.o\
./spiH.o\
./Cap_Spi_Message.o\
./MvDetect.o\
./thread_idle.o\
./Thread_Priority.o\
./MiscCaptureGroup.o\
./BMPCaptureGroup.o\
./ChosenCaptureGroup.o\
./PanoCaptureGroup.o\
./GLEnv.o\
./doubleScreen.o\
./AlarmTarget.o \
./Camera.o \
./CaptureGroup.o \
./CheckMyself.o \
./CornerMarker.o \
./DataofAlarmarea.o \
./DynamicTrack.o \
./ExposureCompensationThread.o \
./FBOManager.o \
./FishCalib.o \
./ForeSight.o \
./GLRender.o \
./HDV4lcap.o \
./LineofRuler.o \
./Loader.o \
./OitVehicle.o \
./PBOManager.o \
./PBO_FBO_Facade.o \
./Parayml.o \
./PresetCameraGroup.o \
./ProcessIPCMsg.o \
./RenderDrawBehvrImpl.o \
./RenderMain.o \
./STLASCIILoader.o \
./SelfCheckThread.o \
./Serial_port.o \
./ShaderParamArrays.o \
./Zodiac_GPIO_Message.o \
./Zodiac_Message.o \
./buffer.o \
./common.o \
./deinterlacing.o \
./glInfo.o \
./glm.o \
./gst_capture.o \
./main.o \
./overLap.o \
./overLapBuffer.o \
./overLapRegion.o \
./pboProcessSrcThread.o \
./recvUARTdata.o \
./recvwheeldata.o \
./scanner.o \
./thread.o \
./updatebuffer.o \
./Xin_IPC_Yuan_Recv_Message.o \
./set_button.o \
./v4l2camera.o 

CPP_DEPS += \
./ClicktoMoveForesight.d\
./spiH.d\
./Cap_Spi_Message.d\
./MvDetect.d\
./thread_idle.d\
./Thread_Priority.d\
./MiscCaptureGroup.d\
./BMPCaptureGroup.d\
./ChosenCaptureGroup.d\
./PanoCaptureGroup.d\
./GLEnv.d\
./doubleScreen.d\
./AlarmTarget.d \
./Camera.d \
./CaptureGroup.d \
./CheckMyself.d \
./CornerMarker.d \
./DataofAlarmarea.d \
./DynamicTrack.d \
./ExposureCompensationThread.d \
./FBOManager.d \
./FishCalib.d \
./ForeSight.d \
./GLRender.d \
./HDV4lcap.d \
./LineofRuler.d \
./Loader.d \
./OitVehicle.d \
./PBOManager.d \
./PBO_FBO_Facade.d \
./Parayml.d \
./PresetCameraGroup.d \
./ProcessIPCMsg.d \
./RenderDrawBehvrImpl.d \
./RenderMain.d \
./STLASCIILoader.d \
./SelfCheckThread.d \
./Serial_port.d \
./ShaderParamArrays.d \
./Zodiac_GPIO_Message.d \
./Zodiac_Message.d \
./buffer.d \
./common.d \
./deinterlacing.d \
./glInfo.d \
./glm.d \
./gst_capture.d \
./main.d \
./overLap.d \
./overLapBuffer.d \
./overLapRegion.d \
./pboProcessSrcThread.d \
./recvUARTdata.d \
./recvwheeldata.d \
./scanner.d \
./thread.d \
./updatebuffer.d \
./Xin_IPC_Yuan_Recv_Message.d \
./set_button.d \
./v4l2camera.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-8.0/bin/nvcc -DDISABLE_NEON_DEI=1 -DNO_ARM_NEON=1 -DENABLE_ENHANCE_FUNCTION=1 -DMVDECT=1 -DUSE_BMPCAP=0 -DWHOLE_PIC=1 -DUSE_GAIN=1 -DCAM_COUNT=10 -DUSE_CAP_SPI=1 -DTRACK_MODE=0 -DDOUBLE_SCREEN=1 -DTEST_GAIN=1 -DGSTREAM_CAP=1 -DUSE_UART=0 -DUSE_12=1 -I../Track/inc -I../OSA_CAP/inc -I../GLTool/include -I/usr/include/opencv -I/usr/include/gstreamer-1.0 -I/usr/include/glib-2.0 -I/usr/lib/aarch64-linux-gnu/glib-2.0/include -I/usr/lib/aarch64-linux-gnu/include -I/usr/include -I/usr/lib/aarch64-linux-gnu/gstreamer-1.0/include -O3 -ccbin aarch64-linux-gnu-g++ -gencode arch=compute_50,code=sm_50 -m64 -odir "." -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-8.0/bin/nvcc -DDISABLE_NEON_DEI=1 -DNO_ARM_NEON=1 -DMVDECT=1 -DENABLE_ENHANCE_FUNCTION=1 -DUSE_BMPCAP=0 -DDOUBLE_SCREEN=1 -DWHOLE_PIC=1 -DUSE_GAIN=1 -DUSE_CAP_SPI=1 -DCAM_COUNT=10 -DTRACK_MODE=0 -DTEST_GAIN=1 -DGSTREAM_CAP=1 -DUSE_UART=0 -DUSE_12=1 -I../Track/inc -I../OSA_CAP/inc -I/usr/include/opencv -I../GLTool/include -I/usr/include/gstreamer-1.0 -I/usr/include/glib-2.0 -I/usr/lib/aarch64-linux-gnu/glib-2.0/include -I/usr/lib/aarch64-linux-gnu/include -I/usr/include -I/usr/lib/aarch64-linux-gnu/gstreamer-1.0/include -O3 --compile -m64 -ccbin aarch64-linux-gnu-g++  -x c++ -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


