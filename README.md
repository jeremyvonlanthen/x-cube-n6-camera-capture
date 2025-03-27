# x-cube-n6-camera-capture Application

Computer Vision application to capture video stream using USB UVC.

Streamed video format is 800x480 YUV422 at 30 frames per second.

## Package Content

This package contains in the Binary directory ready-to-flash Intel hex files for STM32N6570-DK and NUCLEO-N657X0-Q platforms.

For each platform, there are two binary flavors:
- front: It's a selfie-like orientation.
- rear: It's a camera-like orientation.

## Hardware Support

### MB1939 STM32N6570-DK

- The board should be connected to the onboard ST-LINK debug adapter CN6 with a **USB-C to USB-C cable to ensure sufficient power**.
- An additional USB cable to connect USB1 (CN18) to the host computer for UVC streaming.
- OTP fuses are set in this example for xSPI IOs to get the maximum speed (200MHz) on xSPI interfaces.

- 3 cameras are supported:
  - MB1854B IMX335 (Default camera provided with the MB1939 STM32N6570-DK board)
  - STEVAL-55G1MBI VD55G1 camera module
  - STEVAL-66GYMAI VD66GY camera module

### MB1940 NUCLEO-N657X0-Q

- The board should be connected to the onboard ST-LINK debug adapter CN10 with a **USB-C to USB-C cable to ensure sufficient power**.
- An additional USB cable to connect USB (CN8) to the host computer for UVC streaming.
- OTP fuses are set in this example for xSPI IOs to get the maximum speed (200MHz) on xSPI interfaces.

- 3 cameras are supported:
  - MB1854B IMX335
  - STEVAL-55G1MBI VD55G1 camera module
  - STEVAL-66GYMAI VD66GY camera module

## Tools Version

- [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) (**v2.18.0**)

## Console parameters

You can see application messages by attaching a console application to the ST-Link console output. Use the following console parameters:
- Baud rate of 115200 bps.
- No parity.
- One stop bit.

## Flash Binaries

### STM32N6570-DK

1. Set BOOT0 and BOOT1 switches to the right position.
2. Power up the board by connecting the ST-LINK debug adapter CN6 to the host computer.
3. Program `Binary/x-cube-n6-camera-capture_<front|rear>_STM32N6570-DK.hex`.
4. Set BOOT0 and BOOT1 switches to the left position.
5. Perform a power down/up sequence.
6. If not yet done, connect USB1 (CN18) to the host computer.
7. Launch the host webcam application.

### NUCLEO-N657X0-Q

1. Set BOOT0 and BOOT1 switches to the right position.
2. Power up the board by connecting the ST-LINK debug adapter CN10 to the host computer.
3. Program `Binary/x-cube-n6-camera-capture_<front|rear>_NUCLEO-N657X0-Q.hex`.
4. Set BOOT0 and BOOT1 switches to the left position.
5. Perform a power down/up sequence.
6. If not yet done, connect USB (CN8) to the host computer.
7. Launch the host webcam application.
