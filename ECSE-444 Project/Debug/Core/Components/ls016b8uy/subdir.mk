################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Components/ls016b8uy/ls016b8uy.c 

O_SRCS += \
../Core/Components/ls016b8uy/ls016b8uy.o 

OBJS += \
./Core/Components/ls016b8uy/ls016b8uy.o 

C_DEPS += \
./Core/Components/ls016b8uy/ls016b8uy.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Components/ls016b8uy/%.o Core/Components/ls016b8uy/%.su Core/Components/ls016b8uy/%.cyclo: ../Core/Components/ls016b8uy/%.c Core/Components/ls016b8uy/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Components-2f-ls016b8uy

clean-Core-2f-Components-2f-ls016b8uy:
	-$(RM) ./Core/Components/ls016b8uy/ls016b8uy.cyclo ./Core/Components/ls016b8uy/ls016b8uy.d ./Core/Components/ls016b8uy/ls016b8uy.o ./Core/Components/ls016b8uy/ls016b8uy.su

.PHONY: clean-Core-2f-Components-2f-ls016b8uy

