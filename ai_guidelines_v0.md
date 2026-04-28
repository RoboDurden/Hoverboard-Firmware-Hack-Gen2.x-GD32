# This file is a close English translation of the original German prompt:


Hello Codex. I am the developer of the split hoverboard firmware https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x

The firmware code was moved out to https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x-GD32

Here in Visual Code you have its folder structure available. The firmware code is located in the subfolder `HoverBoardGigaDevice/`, and then essentially in `Src/` and `Inc/`. The subfolders `Arduino Examples/` and `PlatformIO Examples/` can also be interesting for you when it comes to controlling the firmware from an ESP32 via UART.

Discussion still happens in https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues

and there is a small wiki at https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/wiki

Your first task is to write `/ai_guidelines_v1md` completely, in English. I only quickly started `/ai_guidelines.md`.

Correction note: the intended file name is `/ai_guidelines_v1.md`, with a dot before `md`.

Now I will tell you in detail what I would like you to trace in the code and then translate into a detailed `ai_guidelines_v1.md`.

---

The purpose of `ai_guidelines_v1.md` is to make it easier for an AI to get started with the quite complex structure of the firmware, but also to make sure that it understands the structure and only changes the code or adds new features in a positive sense.

The firmware was originally a fork of https://github.com/krisstakos/Hoverboard-Firmware-Hack-Gen2.1/ and only supported one special board which, according to the newly introduced Gen2.x or Gen2.t.l classification, is layout `2.1.2`.

By the way, Bluetooth was still supported there: https://github.com/krisstakos/Hoverboard-Firmware-Hack-Gen2.1/blob/main/HoverBoardGigaDevice/Src/commsBluetooth.c

But I later removed that. Maybe at some point you will feel like integrating it again as `RemoteBluetooh.c`.

Correction note: the likely intended spelling is `RemoteBluetooth.c`.

My first big innovation was support for different boards/layouts. For that there is now a subfolder `Inc/defines/` with the many `defines_2-t-l.h` files.

Another big innovation was support for different target MCUs: `gd32f130`, `gd32e230`, and `gd32f103/stm32f103`. So Gen2.x has actually become Gen2.t.l.

A user then added a `platformio.ini`, so that the firmware supports not only Keil, but also PlatformIO.

I am currently working on integration into my online compiler https://pionierland.de/hoverhack/

So currently three platforms and three targets are supported. Maybe later you can also program target `4=MM32SPIN0X`, `5=LKS32`, `6=MM32SPIN25`, and `7=GD32E235`.

Correction note: in the current code there is already partial target-4/MM32SPIN05 material in the Keil project, `target.h`, and `defines_2-4-1.h`, but it should be treated as partial/experimental unless verified.

Now to the structure of the firmware.

I am an absolute supporter of object-oriented programming (OOP) !!!

One of the greatest inventions of the 20th century was neither the atomic bomb nor the car, but OOP. Because only through OOP do complex systems become possible.

That is why I am naturally unhappy that this firmware is still written in old C. Since some layouts only have a 32 kB MCU, converting it to C++ may remain only a dream.

Nevertheless, I have created an object-oriented-like (OOL) structure. For example, there is the "base class" `remote.h`, in which two "virtual functions" are declared, which are then implemented by the many `remoteXY.c` files.

This is not real OOP, because no objects can be dynamically created. Only one "class" `remoteXY.h` may ever be linked with `#ifdef`. But that is quite intentional, because a `remoteXY` is always connected to real hardware, and there is hardly a realistic use case where, for example, you switch from ESP32 to joystick.

You find the same OOL structure with `bldc.h` and `bldcXY.c`. Here too, it makes little sense to compile block commutation, sine, or FOC at the same time into 32 kB in order to be able to switch at runtime.

Now to the include structure of the firmware:

Practically every `src` file first accesses `Inc/defines.h`.

At that "time" the compiler from Keil and PlatformIO already knows the defines `TARGET=1` and `GD32F130`, if target 1 was selected in the Keil dropdown (and then set under `Options for Target`, in the `C/C++ (AC6)` tab, under `Preprocessor Symbols`) or if they are fixed in `platformio.ini` with `build_flags = -D GD32F103 -D TARGET=2`.

My online compiler does not have these pre-definitions, so then `TARGET` is not defined yet. In this way, the user of the online compiler can then set the target at the beginning of `defines.h`:

```c
#ifndef TARGET
	#define TARGET 1	// Makefile will detect target mcu in this line !
	#if TARGET == 1
		#define GD32F130
	#elif TARGET == 2
		#define GD32F103
	#elif TARGET == 3
		#define GD32E230
	#endif
#endif
```

After that the compiler includes `target.h`.

There, a lot of defines/macros are set depending on the target, for example:

```c
#define TARGET_nvic_irq_enable(a, b, c){nvic_irq_enable(a, b, c);}
```

Whenever a `gd32f130` function `xy` (mainly in `setup.c`) did not fit for a new target, I defined a `TARGET_xy` in `target.h` which makes the command fit.

In addition, defines are also created in `target.h` which make the code easier to read and program in Arduino style. For example, the translation from `PA15` to `PORTA` and `PIN15`:

```c
#define PA15	( (uint32_t)GPIOA | 15 )
```

Plus helpers like:

```c
#define digitalWrite(pin,set) gpio_bit_write(pin&0xffffff00U,  (BIT(pin&0xfU) ), set)
```

Back in `defines.h`, it continues with:

```c
#include "../Inc/configSelect.h"
```

`ConfigSelect.h` is only an intermediate step in which a developer can select their own `configDebug.h`, which actually should not be mirrored to GitHub.

In `config.h` (or `configDebug.h`), the end user then sets exactly what kind of firmware they want to compile.

In general, the end user cannot program, so everything must be adjustable for them in this `config.h`.

Most important is:

```c
#define LAYOUT 1
```

That is the board they have, and they can identify it via the "interactive board identifier" at https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/wiki

Based on `TARGET` and `LAYOUT`, the correct `defines_2-t-l-h` is later included in `defines.h`:

```c
	#define STRINGIZE_AUX(a) #a
	#define STRINGIZE(a) STRINGIZE_AUX(a)
	#define INCLUE_FILE(target,version) STRINGIZE(defines/defines_2-target-version.h)
	#include INCLUE_FILE(TARGET , LAYOUT)	// "defines_2-target-version.h"
```

Correction note: the source code currently spells this macro `INCLUE_FILE`, not `INCLUDE_FILE`. The filename pattern is `defines_2-target-version.h`; the text "defines_2-t-l-h" means `defines_2-t-l.h`.

Some layouts exist with slight differences. For that, no new layout ID is assigned. Instead, there is also this in `config.h`:

```c
		#define LAYOUT_SUB 1	// Layout 2.1.7 exisits as 2.1.7.0 and 2.1.7.1
```

Then a `defines2_1-7.h` can access that:

Correction note: the actual filename is `defines_2-1-7.h`.

```c
#ifdef MASTER_OR_SINGLE		// layout 2.2, 2.6 and 2.7 have buzzer on the slave board.
	#if LAYOUT_SUB == 0
		#define HAS_BUZZER
	#endif
#else
	#if LAYOUT_SUB == 1
		#define HAS_BUZZER
	#endif
#endif
```

In `config.h`, the BLDC control method is then defined. Currently there is only:

```c
	//#define BLDC_BC			// old block commutation bldc control
	#define BLDC_SINE			// silent sine-pwm motor control, added 2025 by Robo Durden.
	//#define BLDC_SINE_BOOSTER		// can boost speed by 15% starting from 87% throttle.
```

It continues with:

```c
	#define DRIVING_MODE 0	//  0=pwm, 1=speed in revs/s*1024, 2=torque in NewtonMeter*1024, 3=iOdometer
```

Here, dynamic switching at runtime can make sense, so all `DRIVING_MODE`s are handled by a single PID in `driver.c`. So this is not OOL.

Correction note: mode `0` is raw PWM and does not use PID correction. `driver.c` has one PID controller instance, but separate PID initialization data for modes 1, 2, and 3.

After a few special defines, `config.h` continues with the important place:

```c
	//#define MASTER		// uncomment for MASTER firmware.
	//#define SLAVE			// uncomment for SLAVE firmware.
	#define SINGLE			// uncomment if firmware is for single board and no master-slave dual board setup
```

Gen1 boards (EFeru) have two motors on the control board. With Gen2 there are two split boards, each controlling only one motor. Between the two boards, in the original hoverboard, there is always a master-slave UART connection.

So the firmware must know whether it is `MASTER` or `SLAVE`. But with my newly introduced `RemoteUartBus`, multiple `SINGLE`s can also be connected to one UART bus.

In the following part of `config.h`, one of the many `REMOTE_XY` options is selected. A `remoteXY.c` therefore supplies the control data, essentially speed and steer.

A prominent exception is `REMOTE_AUTODETECT`. `remoteAutodetect.c` is the most complex "remote" and still brushes up against the 32 kB limit. It not only controls the motor, but detects all pins of an unknown board with a simple UI via UART or RTT.

Because there are only a few user settings left for `REMOTE_AUTODETECT`, `REMOTE_AUTODETECT` is defined right at the beginning of `config.h`.

If not, then the user can still select a `PilotXY`:

```c
		//#define PILOT_USER	// uncomment if you want to extend the firmware with custom code :-)
		//#define PILOT_HOVERBIKE	// very experimental pedal detection with chatGpt5 :-/
```

A `pilotXY.c` is the place in the firmware where the user can program their own functionality. For example, reading a pedal-assist detector in order to program an e-bike:

```c
pilotHoverbike.c : 	int8_t iPhotoLNew = digitalRead(PHOTO_L);
setup.c :
		#ifdef PHOTO_L
			pinModePull(PHOTO_L,GPIO_MODE_INPUT,GPIO_PUPD_PULLUP);
		#endif
		#ifdef PHOTO_R
			pinModePull(PHOTO_R,GPIO_MODE_INPUT,GPIO_PUPD_PULLUP);
		#endif
defines_2-1-20.h :
#define PHOTO_L PC15
#define PHOTO_R PA11
```

After that, `config.h` approaches the end with the assignment of the usually two UART ports:

```c
	#if defined(MASTER) || defined(SLAVE)
		#define MASTERSLAVE_USART		1 	// 	1 is usually PA2/PA3 and the original master-slave 4pin header
																		//	0 is usually PB6/PB7 and the empty header close to the flash-header
																		//	2 is usually PB10/PB11 on stm32f103 boards
	#endif
	#if defined(REMOTE_UART) || defined(REMOTE_UARTBUS) || defined(REMOTE_CRSF) || defined(REMOTE_ROS2)
		#define REMOTE_USART				0 	// 	1 is usually PA2/PA3 and the original master-slave 4pin header
																		//	0 is usually PB6/PB7 and the empty header close to the flash-header
																		//	2 is usually PB10/PB11 on stm32f103 boards
	#endif
```

At the very end, only a few universal settings remain:

```c
#ifdef MASTER_OR_SINGLE
	#define INACTIVITY_TIMEOUT 	8        	// Minutes of not driving until poweroff (not very precise)
	#define CELL_LOW_LVL1     3.5       // Gently beeps, show green battery symbol above this Level.
	#define CELL_LOW_LVL2     3.3       // Battery almost empty, show orange battery symbol above this Level. Charge now!
	#define CELL_LOW_DEAD     3.0       // Undervoltage lockout, show red battery symbol above this Level.
#endif
#define DC_CUR_LIMIT     		15        // Motor DC current limit in amps
```

In `defines.h`, it then continues by including the optional `pilot.XY`:

```c
#ifdef PILOT_HOVERBIKE
	#include "PilotHoverbike.h"		// https://youtu.be/ihCpCtgXIRA
#elif defined (PILOT_USER)
	#include "PilotUser.h"		// for your new onboard user code :-)
#else
	#define PILOT_DEFAULT
#endif
```

After that, the correct `defines2_t-l.h`, or `defines2_ad` for `REMOTE_AUTODETECT`, is included.

Correction note: in the current tree these are `defines/defines_2-target-version.h` and `defines/defines_2-ad.h`.

The deep structure of the includes is completed with:

```c
#include "../Inc/setup.h"
#ifdef RTT_REMOTE
	#include "stdio.h"
	#include "../Src/SEGGER_RTT.h"
	#define RTT_PRINTF(size,string,v1) {char s[size];sprintf(s,string,v1);SEGGER_RTT_WriteString(0, s);}
	#define RTT_PRINTF2(size,string,v1,v2) {char s[size];sprintf(s,string,v1,v2);SEGGER_RTT_WriteString(0, s);}
#else
	#define RTT_PRINTF(size,string,v1)
	#define RTT_PRINTF2(size,string,v1,v2)
#endif
```

After that, in `defines.h`, only the selected settings are checked and parameters are initialized.

`main.c` consists mainly of:

- system initialization, which among other things starts the interrupts for ADC and `bldc.c:CalculateBLDC()`.
- the `while(1)` loop, which calls `remoteXY` and `pilotXY`, and then finalizes the final speed and steer for itself and a possible slave.

Correction note: in the current code, the main loop ultimately computes and applies master/slave motor inputs such as `pwmMaster` and `pwmSlave`; `speed` and `steer` are primarily the command values coming from the remote or pilot path.

`bldc.c` consists mainly of the `CalculateBLDC()` routine:

- evaluating the ADC channels for `currentDC` and `batteryVoltage`
- reading the three hall sensors
- applying the PID for the final PWM value
- calling `bldcXY.c` for motor control

Correction note: PID is applied through `Driver()` for speed/torque/position modes. In raw PWM mode, `Driver()` effectively passes the input through.

`it.c` collects all interrupts.

Essentially for the UARTs, ADC, and `bldc.c:CalculateBLDC()`.

Triggering the ADC proved problematic, because `stm32f103` handles it slightly differently than GD32. Across targets, I only got the following to run consistently:

```c
void TARGET_TIMER0_BRK_UP_TRG_COM_IRQHandler(void)
{
	if (timer_interrupt_flag_get(TIMER_BLDC, TIMER_INT_UP))
	{
		static uint8_t interrupt_toggle = 0;	// Static variable to keep track of calls; by Gemini2.5pro
		interrupt_toggle = 1 - interrupt_toggle;	// Invert the toggle on each entry
		if (interrupt_toggle)		// Only execute every second call as libray/hardware will trigger on up AND down, ignoring timerBldc_paramter_struct.alignedmode = TIMER_COUNTER_CENTER_DOWN
		{
			// Fire the ADC trigger first so the sample instant is earlier and
			// more deterministic relative to the PWM valley.
			TARGET_adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
```

---

So, I wrote all of this down only from my memory. Please check all of it in the real code and expand your understanding.

Basically, this already also shows what is planned/wanted for the future, and what other vibe coders should pay attention to.

Please understand the code and write a complete `ai_guidelines_v1.md`, in English.

Do not change any file other than `/ai_guidelines_v1.md` !!!

Correction note: for the present task, the requested output file is `/ai_guidelines_v0.md`; this sentence is preserved here because it belongs to the original prompt.
