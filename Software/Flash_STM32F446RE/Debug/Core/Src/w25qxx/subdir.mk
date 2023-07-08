################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/w25qxx/delay.c \
../Core/Src/w25qxx/driver_w25qxx.c \
../Core/Src/w25qxx/driver_w25qxx_advance.c \
../Core/Src/w25qxx/driver_w25qxx_basic.c \
../Core/Src/w25qxx/driver_w25qxx_interface.c \
../Core/Src/w25qxx/driver_w25qxx_read_test.c \
../Core/Src/w25qxx/driver_w25qxx_register_test.c \
../Core/Src/w25qxx/qspi.c 

OBJS += \
./Core/Src/w25qxx/delay.o \
./Core/Src/w25qxx/driver_w25qxx.o \
./Core/Src/w25qxx/driver_w25qxx_advance.o \
./Core/Src/w25qxx/driver_w25qxx_basic.o \
./Core/Src/w25qxx/driver_w25qxx_interface.o \
./Core/Src/w25qxx/driver_w25qxx_read_test.o \
./Core/Src/w25qxx/driver_w25qxx_register_test.o \
./Core/Src/w25qxx/qspi.o 

C_DEPS += \
./Core/Src/w25qxx/delay.d \
./Core/Src/w25qxx/driver_w25qxx.d \
./Core/Src/w25qxx/driver_w25qxx_advance.d \
./Core/Src/w25qxx/driver_w25qxx_basic.d \
./Core/Src/w25qxx/driver_w25qxx_interface.d \
./Core/Src/w25qxx/driver_w25qxx_read_test.d \
./Core/Src/w25qxx/driver_w25qxx_register_test.d \
./Core/Src/w25qxx/qspi.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/w25qxx/%.o Core/Src/w25qxx/%.su Core/Src/w25qxx/%.cyclo: ../Core/Src/w25qxx/%.c Core/Src/w25qxx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-w25qxx

clean-Core-2f-Src-2f-w25qxx:
	-$(RM) ./Core/Src/w25qxx/delay.cyclo ./Core/Src/w25qxx/delay.d ./Core/Src/w25qxx/delay.o ./Core/Src/w25qxx/delay.su ./Core/Src/w25qxx/driver_w25qxx.cyclo ./Core/Src/w25qxx/driver_w25qxx.d ./Core/Src/w25qxx/driver_w25qxx.o ./Core/Src/w25qxx/driver_w25qxx.su ./Core/Src/w25qxx/driver_w25qxx_advance.cyclo ./Core/Src/w25qxx/driver_w25qxx_advance.d ./Core/Src/w25qxx/driver_w25qxx_advance.o ./Core/Src/w25qxx/driver_w25qxx_advance.su ./Core/Src/w25qxx/driver_w25qxx_basic.cyclo ./Core/Src/w25qxx/driver_w25qxx_basic.d ./Core/Src/w25qxx/driver_w25qxx_basic.o ./Core/Src/w25qxx/driver_w25qxx_basic.su ./Core/Src/w25qxx/driver_w25qxx_interface.cyclo ./Core/Src/w25qxx/driver_w25qxx_interface.d ./Core/Src/w25qxx/driver_w25qxx_interface.o ./Core/Src/w25qxx/driver_w25qxx_interface.su ./Core/Src/w25qxx/driver_w25qxx_read_test.cyclo ./Core/Src/w25qxx/driver_w25qxx_read_test.d ./Core/Src/w25qxx/driver_w25qxx_read_test.o ./Core/Src/w25qxx/driver_w25qxx_read_test.su ./Core/Src/w25qxx/driver_w25qxx_register_test.cyclo ./Core/Src/w25qxx/driver_w25qxx_register_test.d ./Core/Src/w25qxx/driver_w25qxx_register_test.o ./Core/Src/w25qxx/driver_w25qxx_register_test.su ./Core/Src/w25qxx/qspi.cyclo ./Core/Src/w25qxx/qspi.d ./Core/Src/w25qxx/qspi.o ./Core/Src/w25qxx/qspi.su

.PHONY: clean-Core-2f-Src-2f-w25qxx

