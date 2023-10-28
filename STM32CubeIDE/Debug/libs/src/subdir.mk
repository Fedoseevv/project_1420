################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libs/src/algorithm_by_RF.c \
../libs/src/max30102.c \
../libs/src/mpu9250.c 

OBJS += \
./libs/src/algorithm_by_RF.o \
./libs/src/max30102.o \
./libs/src/mpu9250.o 

C_DEPS += \
./libs/src/algorithm_by_RF.d \
./libs/src/max30102.d \
./libs/src/mpu9250.d 


# Each subdirectory must supply rules for building sources it contributes
libs/src/%.o libs/src/%.su libs/src/%.cyclo: ../libs/src/%.c libs/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../../Core/Inc -I../../STM32_WPAN/App -I../../Drivers/STM32WBxx_HAL_Driver/Inc -I../../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/ST/STM32_WPAN -I../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../../Middlewares/ST/STM32_WPAN/utilities -I../../Middlewares/ST/STM32_WPAN/ble/core -I../../Middlewares/ST/STM32_WPAN/ble/core/auto -I../../Middlewares/ST/STM32_WPAN/ble/core/template -I../../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../Utilities/sequencer -I../../Middlewares/ST/STM32_WPAN/ble -I../../Drivers/CMSIS/Include -I"C:/Users/mixai/STM32CubeIDE/workspace_1.12.0/WS2/STM32CubeIDE/libs/inc" -I"C:/Users/mixai/STM32CubeIDE/workspace_1.12.0/WS2/STM32CubeIDE/libs/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-libs-2f-src

clean-libs-2f-src:
	-$(RM) ./libs/src/algorithm_by_RF.cyclo ./libs/src/algorithm_by_RF.d ./libs/src/algorithm_by_RF.o ./libs/src/algorithm_by_RF.su ./libs/src/max30102.cyclo ./libs/src/max30102.d ./libs/src/max30102.o ./libs/src/max30102.su ./libs/src/mpu9250.cyclo ./libs/src/mpu9250.d ./libs/src/mpu9250.o ./libs/src/mpu9250.su

.PHONY: clean-libs-2f-src

