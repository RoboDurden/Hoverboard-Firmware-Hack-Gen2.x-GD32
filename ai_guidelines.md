# RoboDurden/Hoverboard-Firmware-Hack-Gen2.x-GD32 - ai guidelines

Purpose of this mardown file is to help ai understand the firmware in the HoverBoardGigaDevice/ folder (and the Arduino examples in "Arduino Examples/") so it better understands the object-oriented-like structre of the firmware that is still written only in C.
This file also specifies how new code should nicely fit into the concept.

# 1. History

Originally a clone from https://github.com/krisstakos/Hoverboard-Firmware-Hack-Gen2.1

---

# 2. Present

## 2.1 targets
The firmware supports three different mcu targets: gd32f130 , gd32e230 and gd32f103 / stm32f103

## 2.2 platforms
Binaries can be compile with Keil IDE, PlatformIO (Visual Code)
online compiler https://pionierland.de/hoverhack/ is still highly experimental

---

# 3. Future

## 3.1 wishlist:
- BldcFOC.c

## 3.1 pull request
Keeping the firmware clean and structured has highes priority. Additional features must fit nicely as RemoteXY.c or BldcXY.c or PilotXY.c
Changes to main.c , bldc.c or it.c will only be accepted if they optimize the object-oriented-like structure, fix bugs or optimize performance.



---

