# One-wheel proof of concept on hoverboard split boards

This is a fork of [RoboDurden/Hoverboard-Firmware-Hack-Gen2.x-GD32](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x-GD32), the open firmware for "Gen2" hoverboards that use two split motor boards instead of one mainboard. For everything about the original firmware (supported layouts, flashing, remotes), see that repository and its documentation at [RoboDurden/Hoverboard-Firmware-Hack-Gen2.x](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x).

**All changes are on the [`onewheel`](../../tree/onewheel) branch.** The `main` branch is the unmodified original.

## Purpose

This fork is only a proof that two hoverboard split boards can drive two mechanically connected wheels as one self-balancing unit, using a single IMU:

- The **master** board reads its onboard IMU, runs the balance loop and drives its own motor.
- The master sends the resulting motor command to the **slave** board over the master-slave UART.
- The slave drives the second motor from that command. Its own IMU is not used.

It is not a finished, rideable one-wheel firmware.

## What was tested

| Item | Status |
|---|---|
| Board layout | 2.1.20 (GD32F130C8), two boards |
| IMU | Onboard chip marked `AH7A3C`; works with the existing `MPU_6050old` driver on PB6/PB7 |
| Master motor follows the tilt of the master board | Works on the bench, wheel lifted |
| Slave motor follows the tilt of the master board over UART | Works on the bench, wheel lifted |
| Cut-out past 35° pitch and re-engage when level | Works |
| Riding, tuning under load, both motors on one wheel under load | Not tested |
| Other layouts or MCUs | Not tested |

The controller gains are untuned starting values, and the committed configuration is in bench mode (see below).

## What was added

- **`PilotOnewheel`** (`HoverBoardGigaDevice/Src/PilotOnewheel.c`, settings in `Inc/PilotOnewheel.h`): gyro and accelerometer angle filter, PID balance loop, engage when level, cut-out on pitch, roll or IMU errors, duty warning with tiltback.
- **IMU mounting as two numbers**: which IMU axis points up and which points to the tail. A buzzer setup mode (`ONEWHEEL_BEEP_AXIS`) beeps the axis number so no debugger is needed.
- **Direction test** (`ONEWHEEL_TEST_DIR`): runs each motor alone at low power, so the signs can be set before two coupled motors can fight each other.
- **`REMOTE_NONE`**: no remote control; the pilot alone commands the motors.
- **PlatformIO environments** `onewheel_master` and `onewheel_slave`, so one `config.h` builds both firmwares.
- **I2C bus recovery**: if the MCU is reset while the IMU is sending, the IMU holds SDA low; the firmware now clocks the bus free.
- **Slave startup fix**: the slave no longer waits for a power button it does not have.
- **`BUZZER_SILENT`**: mutes the buzzer completely.
- **`ONEWHEEL_BLACKBOX`**: master and slave save a small diagnostic record to flash 20 s and 60 s after power-on, readable later with an ST-Link. Useful when the debugger cannot be connected while the battery and motor are.

## Build and flash

From `HoverBoardGigaDevice/` with PlatformIO:

```
pio run -e onewheel_master
pio run -e onewheel_slave
```

Flash `onewheel_master` to the board whose IMU is used and `onewheel_slave` to the other. Connect the two boards through the master-slave header (USART1, PA2/PA3): TX to RX crossed, plus GND.

## Bring-up order

1. Set `ONEWHEEL_AXIS_UP` and `ONEWHEEL_AXIS_TAIL` in `Inc/PilotOnewheel.h` for how the master board is mounted (use `ONEWHEEL_BEEP_AXIS` to find them).
2. With the wheel lifted, check that tilting the nose down turns the wheel towards the nose. Flip `ONEWHEEL_DIR_MASTER` if not.
3. Check the slave motor the same way with only that motor connected, or with `ONEWHEEL_TEST_DIR`. Flip `ONEWHEEL_DIR_SLAVE` if it turns the other way.
4. Only then run both motors on one wheel.

## Before riding

The committed settings are for bench testing:

| Setting | Committed | For riding |
|---|---|---|
| `ONEWHEEL_BLACKBOX` (`config.h`) | on | Off. Writing flash pauses the motor output briefly. |
| `BUZZER_SILENT` (`config.h`) | on | Off. The beep is the only warning that the motors are near their limit. |
| `ONEWHEEL_KI` | 0 | Needs a value (about 30 as a start) and tuning. |
| `ONEWHEEL_PWM_MAX` | 300 of 1000 | Raise step by step. |

Known limits:

- Layout 2.1.20 has no DC current sensor, so the firmware's current limit does nothing.
- There is no rider detection by default. The board engages whenever it is level. An option for the foot-pad photo sensors exists (`ONEWHEEL_RIDER_SENSOR`) but is untested.
- If the UART link drops, the slave keeps its last command for up to 2 seconds.
- The Keil project file is not updated with the new source files.

Use at your own risk. A self-balancing vehicle that fails can injure you.

## License

GNU General Public License v3, the same as the original project. See [LICENSE](LICENSE).
