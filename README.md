# ⭐️ Real-Time-Embedded-System
This project leverages gyroscopic sensing to monitor wrist tremors, aiming to mitigate tremors associated with Parkinson's disease.
# Hardware Requirements  
1. STM32F429I-Discovery board (includes L3GD20 gyroscope, ILI9341 LCD, touch controller)
2. USB Type-A to Micro-B cable
3. PC running Windows, macOS, or Linux
# Software Dependencies 
1. ARM GCC toolchain or Keil MDK-ARM
2. Mbed CLI / Mbed Studio (if using Mbed OS)
3. STM32CubeIDE (with STM32F4 HAL and BSP support)
# Setup Instructions
1. Install or verify dependencies:  
   Mbed OS:
   ```bash
   pip install mbed-cli
   ```
   ARM GCC: ensure `arm-none-eabi-gcc` is in your `PATH`.  
   STM32CubeIDE: install and add STM32F4 series package via Package Manager.  
2. Configure project parameters in `src/config.h`:  
   `TIME_INTERVAL`: sampling interval in seconds.  
   `RADIUS_X`, `RADIUS_Y`, `RADIUS_Z`: radius of rotation for each axis in centimeters.  
# Build and Flash
1. File → Import → Existing STM32CubeMX Project  
2. Select the project root directory and click Finish  
3. In Project Settings → C/C++ General → Paths and Symbols, add `src/drivers` to the Include Paths  
4. Build → Debug (or Run) to flash via the integrated ST‑LINK
# Notes
1. Ensure all `.c/.h` files under `src/drivers` are included in the build to avoid missing header or linker errors.  
2. Unused drivers can be removed from the project or excluded in your build script.  
3. If you change sensor wiring or replace peripherals, update `gyro.h` and related BSP driver settings accordingly.  
