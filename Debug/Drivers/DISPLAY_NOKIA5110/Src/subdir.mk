################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/DISPLAY_NOKIA5110/Src/user_5110.c 

OBJS += \
./Drivers/DISPLAY_NOKIA5110/Src/user_5110.o 

C_DEPS += \
./Drivers/DISPLAY_NOKIA5110/Src/user_5110.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/DISPLAY_NOKIA5110/Src/%.o Drivers/DISPLAY_NOKIA5110/Src/%.su: ../Drivers/DISPLAY_NOKIA5110/Src/%.c Drivers/DISPLAY_NOKIA5110/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Drivers/DISPLAY_NOKIA5110/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-DISPLAY_NOKIA5110-2f-Src

clean-Drivers-2f-DISPLAY_NOKIA5110-2f-Src:
	-$(RM) ./Drivers/DISPLAY_NOKIA5110/Src/user_5110.d ./Drivers/DISPLAY_NOKIA5110/Src/user_5110.o ./Drivers/DISPLAY_NOKIA5110/Src/user_5110.su

.PHONY: clean-Drivers-2f-DISPLAY_NOKIA5110-2f-Src

