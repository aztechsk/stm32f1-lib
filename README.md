
# stm32f1-lib

The **stm32f1-lib** C library provides an API for controlling microcontroller peripherals.
It supports microcontrollers from the **STM32F1** family. The supported standard peripherals
include AFIO, GPIO, RCC, PWR, EXTINT, TIM, I2C, IWDG, WWDG, DEVID, SERCOM, FSMC, and DMA,
as well as various hardware components connected to the microcontroller, such as buttons,
LEDs, and LEDUI.

### Library Features

- Standardized API for the AZTech framework.
- Implementation of low-level serial communication protocols (e.g., MODBUS).
- Designed for real-time multitasking applications (dependent on FreeRTOS).
- Efficient interrupt handling.
- Utilizes DMA where applicable.
- Communication peripheral instances are represented as C structures with synchronous (blocking)
  `read()` and `write()` I/O operations.
- Supports low-power microcontroller modes—peripheral blocks are disabled when entering sleep mode.
