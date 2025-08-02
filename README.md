# STM32F407xx GPIO Driver Project

## Overview
This project implements a custom GPIO driver for the STM32F407G-DISC1 board. It handles a PD5 button interrupt to toggle an LED on PD12, with fixes for unexpected LD8 (PD15) behavior. Built to learn GPIO, EXTI, NVIC, and schematic interpretation.

## Features
- PD5 interrupt on falling edge.
- PD12 LED toggle via interrupt.
- Debugged LD8 (PD15) default state issue.
- Reusable C driver functions.

## Getting Started
### Prerequisites
- STM32F407G-DISC1 board.
- ARM toolchain (e.g., GCC ARM Embedded).
- STM32CubeIDE or similar IDE.

### Installation
1. Clone the repo: `git clone https://github.com/yourusername/stm32_gpio_driver.git`
2. Open in STM32CubeIDE.
3. Build and flash to the board.

## Usage
- Press the PD5 button to trigger the interrupt.
- Watch PD12 LED toggle; LD8 should stay off.

## Challenges & Solutions
- **Issue**: LD8 (PD15) turned on instead of PD12.
  - **Cause**: Default PD15 high state.
  - **Fix**: Explicit PD15 config to low.
- **Issue**: PD5 interrupt failed.
  - **Cause**: Unenabled SYSCFG clock.
  - **Fix**: Enabled SYSCFG and adjusted EXTI.

## Lessons Learned
- Debugged with ODR, IMR, PR registers.
- Hardware defaults need software resets.
- No hardware faults—software tweaks sufficed.

## Files
- `stm32f407xx.h`: Register definitions.
- `stm32f407xx_gpio_driver.h`: Driver functions.
- `main.c`: Project entry point.

## Contributing
Fork the repo, make improvements, and submit a PR!

## License
MIT License - See `LICENSE` file.

## Contact
Questions? Reach me on LinkedIn or [your email].

#EmbeddedSystems #STM32 #CProgramming
