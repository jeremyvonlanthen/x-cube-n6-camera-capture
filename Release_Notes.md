# Release Notes for x-cube-n6-camera-capture Application

## Purpose

Computer Vision application to capture video stream using USB UVC on STM32N6570-DK and NUCLEO-N657X0-Q boards.

## Software components

| Name                          | Version            | Release notes
|-------------------------------|--------------------| -------------
| Camera Middleware             | v1.5.0             | [release notes](Lib/Camera_Middleware/Release_Notes.md)
| uvcl                          | v3.0.0             | [release notes](Lib/uvcl/Release_Notes.md)
| CMSIS                         | V6.2.0             | [release notes](STM32Cube_FW_N6/Drivers/CMSIS/Documentation/index.html)
| STM32N6xx CMSIS Device        | V1.3.0             | [release notes](STM32Cube_FW_N6/Drivers/CMSIS/Device/ST/STM32N6xx/Release_Notes.html)
| STM32N6xx HAL/LL Drivers      | V1.3.0             | [release notes](STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Release_Notes.html)
| STM32N6570-DK BSP Drivers     | V1.3.0             | [release notes](STM32Cube_FW_N6/Drivers/BSP/STM32N6570-DK/Release_Notes.html)
| STM32N6xx_Nucleo BSP Drivers  | V1.2.0             | [release notes](STM32Cube_FW_N6/Drivers/BSP/STM32N6xx_Nucleo/Release_Notes.html)
| BSP Component aps256xx        | V1.0.6             | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx/Release_Notes.html)
| BSP Component Common          | V7.3.0             | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/Common/Release_Notes.html)
| BSP Component mx66uw1g45g     | V1.1.0             | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/mx66uw1g45g/Release_Notes.html)
| BSP Component rk050hr18       | V1.0.1             | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/rk050hr18/Release_Notes.html)
| FreeRTOS kernel               | v10.6.2            | [release notes](Lib/FreeRTOS/Source/History.txt)
| Azure RTOS USBX               | V6.4.0             | [release notes](STM32Cube_FW_N6/Middlewares/ST/usbx/README.md)
|                               | ST modified 251022 | [ST release notes](STM32Cube_FW_N6/Middlewares/ST/usbx/st_readme.txt)

## Update history

### V2.0.0 / January 2026

- Add support for multiple streams
- Deliver source code
- Update the STM32CubeN6 Firmware Package to version 1.3.0.
- Add STEVAL-1943-MC1 camera module support.

### V1.0.0 / May 2025

Initial Version
