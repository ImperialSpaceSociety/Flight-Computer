################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/delay.c \
../Core/Src/driver_w25qxx.c \
../Core/Src/driver_w25qxx_advance.c \
../Core/Src/driver_w25qxx_basic.c \
../Core/Src/driver_w25qxx_interface.c \
../Core/Src/driver_w25qxx_read_test.c \
../Core/Src/driver_w25qxx_register_test.c \
../Core/Src/main.c \
../Core/Src/spi.c \
../Core/Src/stm32f3xx_hal_msp.c \
../Core/Src/stm32f3xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f3xx.c \
../Core/Src/uart.c 

OBJS += \
./Core/Src/delay.o \
./Core/Src/driver_w25qxx.o \
./Core/Src/driver_w25qxx_advance.o \
./Core/Src/driver_w25qxx_basic.o \
./Core/Src/driver_w25qxx_interface.o \
./Core/Src/driver_w25qxx_read_test.o \
./Core/Src/driver_w25qxx_register_test.o \
./Core/Src/main.o \
./Core/Src/spi.o \
./Core/Src/stm32f3xx_hal_msp.o \
./Core/Src/stm32f3xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f3xx.o \
./Core/Src/uart.o 

C_DEPS += \
./Core/Src/delay.d \
./Core/Src/driver_w25qxx.d \
./Core/Src/driver_w25qxx_advance.d \
./Core/Src/driver_w25qxx_basic.d \
./Core/Src/driver_w25qxx_interface.d \
./Core/Src/driver_w25qxx_read_test.d \
./Core/Src/driver_w25qxx_register_test.d \
./Core/Src/main.d \
./Core/Src/spi.d \
./Core/Src/stm32f3xx_hal_msp.d \
./Core/Src/stm32f3xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f3xx.d \
./Core/Src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303x8 -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/delay.cyclo ./Core/Src/delay.d ./Core/Src/delay.o ./Core/Src/delay.su ./Core/Src/driver_w25qxx.cyclo ./Core/Src/driver_w25qxx.d ./Core/Src/driver_w25qxx.o ./Core/Src/driver_w25qxx.su ./Core/Src/driver_w25qxx_advance.cyclo ./Core/Src/driver_w25qxx_advance.d ./Core/Src/driver_w25qxx_advance.o ./Core/Src/driver_w25qxx_advance.su ./Core/Src/driver_w25qxx_basic.cyclo ./Core/Src/driver_w25qxx_basic.d ./Core/Src/driver_w25qxx_basic.o ./Core/Src/driver_w25qxx_basic.su ./Core/Src/driver_w25qxx_interface.cyclo ./Core/Src/driver_w25qxx_interface.d ./Core/Src/driver_w25qxx_interface.o ./Core/Src/driver_w25qxx_interface.su ./Core/Src/driver_w25qxx_read_test.cyclo ./Core/Src/driver_w25qxx_read_test.d ./Core/Src/driver_w25qxx_read_test.o ./Core/Src/driver_w25qxx_read_test.su ./Core/Src/driver_w25qxx_register_test.cyclo ./Core/Src/driver_w25qxx_register_test.d ./Core/Src/driver_w25qxx_register_test.o ./Core/Src/driver_w25qxx_register_test.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32f3xx_hal_msp.cyclo ./Core/Src/stm32f3xx_hal_msp.d ./Core/Src/stm32f3xx_hal_msp.o ./Core/Src/stm32f3xx_hal_msp.su ./Core/Src/stm32f3xx_it.cyclo ./Core/Src/stm32f3xx_it.d ./Core/Src/stm32f3xx_it.o ./Core/Src/stm32f3xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f3xx.cyclo ./Core/Src/system_stm32f3xx.d ./Core/Src/system_stm32f3xx.o ./Core/Src/system_stm32f3xx.su ./Core/Src/uart.cyclo ./Core/Src/uart.d ./Core/Src/uart.o ./Core/Src/uart.su

.PHONY: clean-Core-2f-Src

