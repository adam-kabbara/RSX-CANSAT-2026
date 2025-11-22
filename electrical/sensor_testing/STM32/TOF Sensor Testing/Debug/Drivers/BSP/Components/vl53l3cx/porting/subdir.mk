################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform.c \
../Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform_ipp.c \
../Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform_log.c 

C_DEPS += \
./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform.d \
./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform_ipp.d \
./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform_log.d 

OBJS += \
./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform.o \
./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform_ipp.o \
./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform_log.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/vl53l3cx/porting/%.o Drivers/BSP/Components/vl53l3cx/porting/%.su Drivers/BSP/Components/vl53l3cx/porting/%.cyclo: ../Drivers/BSP/Components/vl53l3cx/porting/%.c Drivers/BSP/Components/vl53l3cx/porting/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_32 -DUSE_HAL_DRIVER -DSTM32G431xx -DUSE_NUCLEO_64 -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../TOF/App -I../TOF/Target -I../Drivers/BSP/Components/vl53l3cx/modules -I../Drivers/BSP/Components/vl53l3cx/porting -I../Drivers/BSP/Components/vl53l3cx -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-vl53l3cx-2f-porting

clean-Drivers-2f-BSP-2f-Components-2f-vl53l3cx-2f-porting:
	-$(RM) ./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform.cyclo ./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform.d ./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform.o ./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform.su ./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform_ipp.cyclo ./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform_ipp.d ./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform_ipp.o ./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform_ipp.su ./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform_log.cyclo ./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform_log.d ./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform_log.o ./Drivers/BSP/Components/vl53l3cx/porting/vl53lx_platform_log.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-vl53l3cx-2f-porting

