################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/GPIO_module/user_button.c \
../Drivers/GPIO_module/user_led.c 

OBJS += \
./Drivers/GPIO_module/user_button.o \
./Drivers/GPIO_module/user_led.o 

C_DEPS += \
./Drivers/GPIO_module/user_button.d \
./Drivers/GPIO_module/user_led.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/GPIO_module/%.o Drivers/GPIO_module/%.su Drivers/GPIO_module/%.cyclo: ../Drivers/GPIO_module/%.c Drivers/GPIO_module/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/GPIO_module -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-GPIO_module

clean-Drivers-2f-GPIO_module:
	-$(RM) ./Drivers/GPIO_module/user_button.cyclo ./Drivers/GPIO_module/user_button.d ./Drivers/GPIO_module/user_button.o ./Drivers/GPIO_module/user_button.su ./Drivers/GPIO_module/user_led.cyclo ./Drivers/GPIO_module/user_led.d ./Drivers/GPIO_module/user_led.o ./Drivers/GPIO_module/user_led.su

.PHONY: clean-Drivers-2f-GPIO_module

