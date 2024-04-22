
# stm32f1-lib

The C library **stm32f1-lib** provides an API for controlling the peripherals of the microcontroller.
The supported devices include microcontrollers from **STM32F1** family.
The supported standard peripherals include AFIO, GPIO, RCC, PWR, EXTINT, TIM, I2C, IWDG, WWDG, DEVID, SERCOM, FSMC, DMA and various hardware components
connected to the microcontroller such as buttons, LEDs, LEDUI, etc.

### Library features

- Standardized API (for the AZTech framework).
- Implementation of low-level serial communication protocols.
- Designed for real-time multitasking applications (dependent on FreeRTOS).
- Efficient interrupt handling.
- Use of DMA where appropriate.
- Communication peripheral instances are represented by C structures with synchronous (blocking) read() and write() I/O operations.
- Support for low-power microcontroller modes. Peripheral blocks are turned off when entering sleep mode.
