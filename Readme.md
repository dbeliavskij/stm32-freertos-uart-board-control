# STM32 board control via UART using FreeRTOS

*Program is written for Nucleo-L476rg microcontroller*

## Short program description

Using this program, you can control what functions your STM32 board performs by executing commands through serial terminal

## Features

- All commands are case insensitive
- Blink on board LED with set period

Type `LED b XXXX`, where `XXXX` represents blinking period in ms and is any number from 1 to 9999.
To turn off this feature type `LED b XXXX`, where `XXXX` is any letter or number less than 1

**Important to note!** LED blinking task automatically disables Button LED control feature described later.
- Turn on or off onboard LED through console

Type `LED u 1` to turn LED on and `LED u XXXX` to turn off, where `XXXX` is any other number and/or symbol combination.

**Important to note!** UART LED control automatically disables LED blinking task

- Button press recorder

Type `but r 1` to enable notifications through UART that User button on the board was pressed. To turn off notifications, type `but r XXXX`, where `XXXX` is any other number and/or symbol combination.

- Control onboard LED with User button

Type `but l 1` and each time User button will be pressed, it will toggle onboard LED, To disable, type `but l XXXX`, where `XXXX` is any other number and/or symbol combination.

**Important to note!** Button LED control automatically disables LED blinking task

## Requirements for entered commands

Command lenght is 7-10 symbols, longer or shorter commands will not be accepted or will be automatically modified.

Command processing will start if user presses `Enter` or no input is received via UART for more than 500ms