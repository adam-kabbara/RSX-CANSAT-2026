################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Libraries/Src/BMP581.cpp \
../Libraries/Src/BNO085.cpp \
../Libraries/Src/INA219.cpp \
../Libraries/Src/VL53L1X_api.cpp \
../Libraries/Src/VL53L1X_calibration.cpp \
../Libraries/Src/servo.cpp \
../Libraries/Src/vl53l1_platform.cpp 

OBJS += \
./Libraries/Src/BMP581.o \
./Libraries/Src/BNO085.o \
./Libraries/Src/INA219.o \
./Libraries/Src/VL53L1X_api.o \
./Libraries/Src/VL53L1X_calibration.o \
./Libraries/Src/servo.o \
./Libraries/Src/vl53l1_platform.o 

CPP_DEPS += \
./Libraries/Src/BMP581.d \
./Libraries/Src/BNO085.d \
./Libraries/Src/INA219.d \
./Libraries/Src/VL53L1X_api.d \
./Libraries/Src/VL53L1X_calibration.d \
./Libraries/Src/servo.d \
./Libraries/Src/vl53l1_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/Src/%.o Libraries/Src/%.su Libraries/Src/%.cyclo: ../Libraries/Src/%.cpp Libraries/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_NUCLEO_32 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Libraries/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Libraries-2f-Src

clean-Libraries-2f-Src:
	-$(RM) ./Libraries/Src/BMP581.cyclo ./Libraries/Src/BMP581.d ./Libraries/Src/BMP581.o ./Libraries/Src/BMP581.su ./Libraries/Src/BNO085.cyclo ./Libraries/Src/BNO085.d ./Libraries/Src/BNO085.o ./Libraries/Src/BNO085.su ./Libraries/Src/INA219.cyclo ./Libraries/Src/INA219.d ./Libraries/Src/INA219.o ./Libraries/Src/INA219.su ./Libraries/Src/VL53L1X_api.cyclo ./Libraries/Src/VL53L1X_api.d ./Libraries/Src/VL53L1X_api.o ./Libraries/Src/VL53L1X_api.su ./Libraries/Src/VL53L1X_calibration.cyclo ./Libraries/Src/VL53L1X_calibration.d ./Libraries/Src/VL53L1X_calibration.o ./Libraries/Src/VL53L1X_calibration.su ./Libraries/Src/servo.cyclo ./Libraries/Src/servo.d ./Libraries/Src/servo.o ./Libraries/Src/servo.su ./Libraries/Src/vl53l1_platform.cyclo ./Libraries/Src/vl53l1_platform.d ./Libraries/Src/vl53l1_platform.o ./Libraries/Src/vl53l1_platform.su

.PHONY: clean-Libraries-2f-Src

