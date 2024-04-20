# iFlight BLITZ H7 Pro Flight Controller

https://shop.iflight-rc.com/BLITZ-Whoop-F7-AIO-Pro1927

The Blitz H7 Pro is a flight controller produced by [iFlight](https://shop.iflight-rc.com/).

## Features

 - MCU - STM32H743 32-bit processor running at 480 MHz
 - Gyro: ICM42688
 - SDCard for logging
 - BEC output: 5V 2.5A
 - Barometer: DPS310
 - OSD: AT7456E
 - 7x UARTs
 - 13x PWM Outputs (12 Motor Output, 1 LED)
 - Battery input voltage: 2S-6S
 - I2C for external compass.

## Pinout

![BLITZ H7 Pro Board](blitz_f7_pinout.jpg "BLITZ H7 Pro")

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.
|Name|Pin|Function|
|:-|:-|:-|
|SERIAL0|COMPUTER|USB|
|SERIAL1|RX1/TX1|UART1 (DJI connector, DMA-enabled)|
|SERIAL2|TX2/RX2|UART2 (RX, DMA-enabled)|
|SERIAL3|TX3/RX3|UART3 (DMA-enabled)|
|SERIAL4|TX4/RX4|UART4 (GPS, DMA-enabled)|
|SERIAL5|RX5|UART5 (ESC Telemetry)|
|SERIAL6|TX6/RX6|UART6|
|SERIAL7|TX7/RX7|UART7|

## RC Input

RC input is configured on the (UART2_RX/UART2_TX) pins which forms part of the DJI connector. It supports all serial RC protocols.

## OSD Support

The BLITZ H7 Pro supports OSD using OSD_TYPE 1 (MAX7456 driver).

## PWM Output

The BLITZ H7 Pro has 12 PWM outputs. The pads for motor output M1-M4 are in one ESC connector and M5-M8 in the second ESC connector. The remaining outputs are on the pads on the daughterboard. The first 8 outputs support bi-directional DShot and DShot, as well as all PWM types.

The PWM are in in two groups:

 - PWM 1-2 in group1
 - PWM 3-6 in group2
 - PWM 7-8 in group3
 - PWM 9   in group4

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot.

## Battery Monitoring

The board has a builtin voltage sensor and a current sensor input tied to its 4 in 1 ESC current sensor. The voltage sensor can handle up to 6S
LiPo batteries.

The correct battery setting parameters are:

 - BATT_MONITOR 4
 - BATT_VOLT_PIN 10
 - BATT_VOLT_MULT 11
 - BATT_CURR_PIN 11
 - BATT_CURR_MULT 50

These are set by default in the firmware and shouldn't need to be adjusted

## Compass

The BLITZ H7 Pro does not have a builtin compass, but you can attach an external compass to I2C pins.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.
