################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/mpu6050_driver/mpu6050.c 

OBJS += \
./Drivers/mpu6050_driver/mpu6050.o 

C_DEPS += \
./Drivers/mpu6050_driver/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/mpu6050_driver/%.o Drivers/mpu6050_driver/%.su Drivers/mpu6050_driver/%.cyclo: ../Drivers/mpu6050_driver/%.c Drivers/mpu6050_driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/mpu6050_driver -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-mpu6050_driver

clean-Drivers-2f-mpu6050_driver:
	-$(RM) ./Drivers/mpu6050_driver/mpu6050.cyclo ./Drivers/mpu6050_driver/mpu6050.d ./Drivers/mpu6050_driver/mpu6050.o ./Drivers/mpu6050_driver/mpu6050.su

.PHONY: clean-Drivers-2f-mpu6050_driver

