################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Components/m24sr/m24sr.c 

O_SRCS += \
../Core/Components/m24sr/m24sr.o 

OBJS += \
./Core/Components/m24sr/m24sr.o 

C_DEPS += \
./Core/Components/m24sr/m24sr.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Components/m24sr/%.o Core/Components/m24sr/%.su Core/Components/m24sr/%.cyclo: ../Core/Components/m24sr/%.c Core/Components/m24sr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Components-2f-m24sr

clean-Core-2f-Components-2f-m24sr:
	-$(RM) ./Core/Components/m24sr/m24sr.cyclo ./Core/Components/m24sr/m24sr.d ./Core/Components/m24sr/m24sr.o ./Core/Components/m24sr/m24sr.su

.PHONY: clean-Core-2f-Components-2f-m24sr

