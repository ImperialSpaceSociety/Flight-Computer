This is a project using Platform.io for use on the NUCLEO-32 dev board. The chip is the STM32F303K8. More information on setup with Platform.io can be found at [here](https://docs.platformio.org/en/stable/boards/ststm32/nucleo_f303k8.html#board-ststm32-nucleo-f303k8)

1) Download the Platform.io extension on VS code
2) Create a project on platform.io using the STMCube framework
3) Create another project using STM32CubeMX for this board and generate code using the default settings.
4) Copy the src .h files to the include folder, libraries to the lib folder and src .c files to the src folder

This blink example uses code from https://github.com/platformio/platform-ststm32/blob/master/examples/stm32cube-hal-blink/src/main.c

However, the pinout has been changed to pin: gpio_pin_3 and port: gpiob as determined from the datasheet.