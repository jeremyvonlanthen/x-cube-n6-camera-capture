# x-cube-n6-camera-capture Application

Computer Vision application to capture video stream using USB UVC on the STM32N6570-DK or NUCLEO-N657X0-Q board.

Streamed video format can be dynamically selected among :

- 224x224 YUV422 at 30 frames per second
- 256x256 YUV422 at 30 frames per second
- 480x480 YUV422 at 30 frames per second
- 640x480 YUV422 at 30 frames per second
- 800x480 YUV422 at 30 frames per second

---

## Doc Folder Content

- [Boot Overview](Doc/Boot-Overview.md)
- [Configure available streams](Doc/Build-Options.md)

---

## Features Demonstrated in This Example

- Multi-threaded application flow (FreeRTOS)
- DCMIPP crop, decimation, downscale
- DCMIPP ISP usage
- USB UVC (Azure RTOS USBX)

---

## Package Content

This package contains in the Binary directory ready-to-flash Intel hex files for STM32N6570-DK and NUCLEO-N657X0-Q platforms.

For each platform, there are two binary flavors:

- front: It's a selfie-like orientation.
- rear: It's a camera-like orientation.

## Hardware Support

Supported development platforms:

- [STM32N6570-DK](https://www.st.com/en/evaluation-tools/stm32n6570-dk.html) Discovery Board
  - Connect to the onboard ST-LINK debug adapter (CN6) using a __USB-C to USB-C cable__ for sufficient power.
  - An additional USB cable to connect USB1 (CN18) to the host computer for UVC streaming.
  - OTP fuses are configured for xSPI IOs to achieve maximum speed (200MHz) on xSPI interfaces.
- [NUCLEO-N657X0-Q](https://www.st.com/en/evaluation-tools/nucleo-n657x0-q.html) Nucleo Board
  - Connect to the onboard ST-LINK debug adapter (CN10) using a __USB-C to USB-C cable__ for sufficient power.
  - An additional USB cable to connect USB1 (CN8) to the host computer for UVC streaming.
  - OTP fuses are configured for xSPI IOs to achieve maximum speed (200MHz) on xSPI interfaces.

![Board](_htmresc/STM32N6570-DK.png)
STM32N6570-DK board with MB1854B IMX335.

![Board](_htmresc/NUCLEO-N657X0-Q_USB_UVC.png)
NUCLEO-N657X0-Q board with MB1854B IMX335.

Supported camera modules:

- IMX335 camera module
- [STEVAL-55G1MBI](https://www.st.com/en/evaluation-tools/steval-55g1mbi.html)
- [STEVAL-66GYMAI1](https://www.st.com/en/evaluation-tools/steval-66gymai.html)
- [STEVAL-1943-MC1](https://www.st.com/en/evaluation-tools/steval-1943-mc1.html)

---

## Tools Version

- IAR Embedded Workbench for Arm (__EWARM 9.40.1__) + N6 patch ([__EWARMv9_STM32N6xx_V1.0.0__](STM32Cube_FW_N6/Utilities/PC_Software/EWARMv9_STM32N6xx_V1.0.0.zip))
- [STM32CubeIDE](https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-ides/stm32cubeide.html) (__v1.17.0__)
- [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) (__v2.18.0__)

---

## Boot Modes

The STM32N6 series does not have internal flash memory. To retain firmware after a reboot, program it into the external flash. Alternatively, you can load firmware directly into SRAM (development mode), but note that the program will be lost if the board is powered off in this mode.

Development Mode: used for loading firmware into RAM during a debug session or for programming firmware into external flash.

Boot from Flash: used to boot firmware from external flash.

|                  | STM32N6570-DK                                                                | NUCLEO-N657X0-Q                                                                      |
| -------------    | -------------                                                                |-----------------                                                                     |
| Boot from flash  | ![STM32N6570-DK Boot from flash](_htmresc/STM32N6570-DK_Boot_from_flash.png) | ![NUCLEO-N657X0-Q Boot from flash](_htmresc/NUCLEO-N657X0-Q_Boot_from_flash.png)     |
| Development mode | ![STM32N6570-DK Development mode](_htmresc/STM32N6570-DK_Dev_mode.png)       | ![NUCLEO-N657X0-Q Development mode](_htmresc/NUCLEO-N657X0-Q_Dev_mode.png)           |

---

## Console parameters

You can see application messages by attaching a console application to the ST-Link console output. Use the following console parameters:
- Baud rate of 115200 bps.
- No parity.
- One stop bit.

---

## Quickstart Using Prebuilt Binaries

### Flash Prebuilt Binaries

#### STM32N6570-DK

  1. Set the board to [development mode](#boot-modes).
  2. Program either `Binary/STM32N6570-DK/x-cube-n6-camera-capture-front-dk.hex` or `Binary/STM32N6570-DK/x-cube-n6-camera-capture-rear-dk.hex`
  3. Set the board to [boot from flash mode](#boot-modes).
  4. Power cycle the board.
  5. Check that the additional USB cable between USB1 (CN18) and the host computer is connected.
  6. Launch your favorite webcam application

#### NUCLEO-N657X0-Q

  1. Set the board to [development mode](#boot-modes).
  2. Program either `Binary/NUCLEO-N657X0-Q/x-cube-n6-camera-capture-front-nucleo.hex` or `Binary/NUCLEO-N657X0-Q/x-cube-n6-camera-capture-rear-nucleo.hex`
  3. Set the board to [boot from flash mode](#boot-modes).
  4. Power cycle the board.
  5. Check that the additional USB cable between USB1 (CN8) and the host computer is connected.
  6. Launch your favorite webcam application

---

### How to Program Hex Files Using STM32CubeProgrammer UI

See [How to Program Hex Files STM32CubeProgrammer](Doc/Program-Hex-Files-STM32CubeProgrammer.md).

---

### How to Program Hex Files Using Command Line

Make sure to have the STM32CubeProgrammer bin folder added to your path.
Instructions below are for STM32N6570-DK. For NUCLEO-N657X0-Q you have to select one of the two nucleo project according to your use case.

```bash
export DKEL="<STM32CubeProgrammer_N6 Install Folder>/bin/ExternalLoader/MX66UW1G45G_STM32N6570-DK.stldr"

# Application Firmware
STM32_Programmer_CLI -c port=SWD mode=HOTPLUG -el $DKEL -hardRst -w Binary/STM32N6570-DK/x-cube-n6-camera-capture-front-dk.hex
```

---

## Quickstart Using Source Code

More information about boot modes is available at [Boot Overview](Doc/Boot-Overview.md).

### Application Build and Run - Dev Mode

Set your board to [development mode](#boot-modes).

Instructions below are for STM32N6570-DK. Select nucleo project for NUCLEO-N657X0-Q.

#### STM32CubeIDE

Double click on `STM32CubeIDE/STM32N6570-DK/.project` to open the project in STM32CubeIDE. Build and run with the build and run buttons.

#### IAR EWARM

Double click on `EWARM/STM32N6570-DK/x-cube-n6-camera-capture-dk.eww` to open the project in IAR IDE. Build and run with the build and run buttons.

#### Makefile

Before running the commands below, be sure to have the commands in your PATH.

1. Build the project using the provided `Makefile`:

```bash
make -j8
```

2. Open a GDB server connected to the STM32 target:

```bash
ST-LINK_gdbserver -p 61234 -l 1 -d -s -cp <path-to-stm32cubeprogramer-bin-dir> -m 1 -g
```

3. In a separate terminal session, launch a GDB session to load the firmware image into the device memory:

```bash
$ arm-none-eabi-gdb build/Project.elf
(gdb) target remote :61234
(gdb) monitor reset
(gdb) load
(gdb) continue
```

---

### Application Build and Run - Boot from Flash

Set your board to [development mode](#boot-modes).

Instructions below are for STM32N6570-DK. Select nucleo project for NUCLEO-N657X0-Q.

#### STM32CubeIDE

Double click on `STM32CubeIDE/STM32N6570-DK/.project` to open project in STM32CubeIDE. Build with build button.

#### IAR EWARM

Double click on `EWARM/STM32N6570-DK/x-cube-n6-camera-capture-dk.eww` to open project in IAR IDE. Build with build button.

#### Makefile

Before running the commands below, be sure to have them in your PATH.

1. Build project using the provided `Makefile`:

```bash
make -j8
```

Once your app is built with Makefile, STM32CubeIDE, or EWARM, you must add a signature to the bin file:
```bash
STM32_SigningTool_CLI -bin build/Project.bin -nk -t ssbl -hv 2.3 -o build/Project_sign.bin
```

You can program the signed bin file at the address `0x70000000`.

```bash
export DKEL="<STM32CubeProgrammer_N6 Install Folder>/bin/ExternalLoader/MX66UW1G45G_STM32N6570-DK.stldr"

# Adapt build path to your IDE
STM32_Programmer_CLI -c port=SWD mode=HOTPLUG -el $DKEL -hardRst -w build/Project_sign.bin 0x70000000
```

Set your board to [boot from flash](#boot-modes) mode and power cycle to boot from external flash.

## Known Issues

- Stability issues may occur when changing the stream. Your webcam application may report an error. In that case,
  restart the camera capture application.
