################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ECUAL/LCD16X2/LCD16X2.c \
../ECUAL/LCD16X2/LCD16X2_cfg.c 

OBJS += \
./ECUAL/LCD16X2/LCD16X2.o \
./ECUAL/LCD16X2/LCD16X2_cfg.o 

C_DEPS += \
./ECUAL/LCD16X2/LCD16X2.d \
./ECUAL/LCD16X2/LCD16X2_cfg.d 


# Each subdirectory must supply rules for building sources it contributes
ECUAL/LCD16X2/%.o ECUAL/LCD16X2/%.su ECUAL/LCD16X2/%.cyclo: ../ECUAL/LCD16X2/%.c ECUAL/LCD16X2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ECUAL-2f-LCD16X2

clean-ECUAL-2f-LCD16X2:
	-$(RM) ./ECUAL/LCD16X2/LCD16X2.cyclo ./ECUAL/LCD16X2/LCD16X2.d ./ECUAL/LCD16X2/LCD16X2.o ./ECUAL/LCD16X2/LCD16X2.su ./ECUAL/LCD16X2/LCD16X2_cfg.cyclo ./ECUAL/LCD16X2/LCD16X2_cfg.d ./ECUAL/LCD16X2/LCD16X2_cfg.o ./ECUAL/LCD16X2/LCD16X2_cfg.su

.PHONY: clean-ECUAL-2f-LCD16X2

