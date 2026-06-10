################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Lib/command_manager.cpp \
../Core/Lib/controller.cpp \
../Core/Lib/mission_manager.cpp \
../Core/Lib/sensor_manager.cpp \
../Core/Lib/serial_manager.cpp \
../Core/Lib/telemetry_manager.cpp 

OBJS += \
./Core/Lib/command_manager.o \
./Core/Lib/controller.o \
./Core/Lib/mission_manager.o \
./Core/Lib/sensor_manager.o \
./Core/Lib/serial_manager.o \
./Core/Lib/telemetry_manager.o 

CPP_DEPS += \
./Core/Lib/command_manager.d \
./Core/Lib/controller.d \
./Core/Lib/mission_manager.d \
./Core/Lib/sensor_manager.d \
./Core/Lib/serial_manager.d \
./Core/Lib/telemetry_manager.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/%.o Core/Lib/%.su Core/Lib/%.cyclo: ../Core/Lib/%.cpp Core/Lib/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_NUCLEO_32 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Libraries/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib

clean-Core-2f-Lib:
	-$(RM) ./Core/Lib/command_manager.cyclo ./Core/Lib/command_manager.d ./Core/Lib/command_manager.o ./Core/Lib/command_manager.su ./Core/Lib/controller.cyclo ./Core/Lib/controller.d ./Core/Lib/controller.o ./Core/Lib/controller.su ./Core/Lib/mission_manager.cyclo ./Core/Lib/mission_manager.d ./Core/Lib/mission_manager.o ./Core/Lib/mission_manager.su ./Core/Lib/sensor_manager.cyclo ./Core/Lib/sensor_manager.d ./Core/Lib/sensor_manager.o ./Core/Lib/sensor_manager.su ./Core/Lib/serial_manager.cyclo ./Core/Lib/serial_manager.d ./Core/Lib/serial_manager.o ./Core/Lib/serial_manager.su ./Core/Lib/telemetry_manager.cyclo ./Core/Lib/telemetry_manager.d ./Core/Lib/telemetry_manager.o ./Core/Lib/telemetry_manager.su

.PHONY: clean-Core-2f-Lib

