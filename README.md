# Pico DShot


## Notes on setting up the repo

- git clone repo
- git submodule init
- git submodule update
- `cd lib/extern/pico-sdk; git submodule update --init` <-- This was required for `TinyUSB`
This repo is being developed to use a RPi Pico to send dshot commands to ESCs.
This is a work in progress.

---
## Code Overview

```
|-- lib
    |-- dshot
    |-- config
    |-- tts
    |-- shoot
    |-- utils
|-- src
    | -- main.cpp
|-- test
    | -- test_dshot.cpp
```

- `lib/` contains the libraries used to implement dshot on the pico
    - `dshot/` is a standalone module that is used to convert a dshot command to a frame that can be sent using pwm hardware
    - `config/` a header file for dshot globals, which are used to configure the hardware in `tts/`
    - `tts/` contains the hardware implementations for dma, pwm, alarm pool
    - `shoot/` implements sending dshot commands using the pwm hw as well as a repeating timer. A future merge request will add telem over uart in this module
    - `utils/` just an led flashing for debugging

- `src/` contains `main.cpp` which uses serial input to send dshot commands
- `test/` contains a unit test file `test_dshot.cpp`

Dependency Graph:

```
|-- shoot
|   |-- tts
|   |   |-- config
|   |-- dshot
|-- utils
```

## To Do
- [ ] Convert `dshot/` module to just a header file using static inlines (make sure to check it works with the unit tests)
- [ ] Move `config.h` to `tts/dshotglobals.h` . Wrap variables in a namespace and remove prefix `DSHOT_` (a bit difficult because dshot namespace already exists)
- [ ] `tts.h` can be renamed `dshothw.h`?
- [ ] Maybe change / clarify the use of *code* and *command*

## Backlog
- [ ] attempt proper arm sequence
- [ ] Try: `DSHOT_SPEED = DEBUG ? 0.008 : 1200 // kHz` (this may get rid of some other ternaries on DEBUG)
- [ ] Add validation to ensure PWM, DMA, repeating timer have been setup correctly
- [ ] Currently dma writes to a PWM counter compare. This actually writes to two dma channels (because upper / lower 16 bits are separate counters). Hence we render one dma channel useless. Is it possible to implement this in a better way?
- [ ] Do we need to use the Arduino framework? Or can we just use the Pico SDK and import libraries 3rd party libs if necessary? If the latter, we could either consider [Wiz IO](https://github.com/Wiz-IO/wizio-pico) or check out [this post](https://community.platformio.org/t/include-pico-stdlib-h-causes-errors/22997). 
- [ ] Transfer print_*_setup functions to logging / utils? Maybe check `refactor` branch. Maybe include the serial printf lib. 

---
## DShot Protocol

A flight controller (Pico) sends commands to the ESC to control the motor.
DShot is a digital protocol which typically sends commands using PWM hardware (note that we are *not* sending PWM frames, but are rather piggy backing off of its hardware to send dshot frames!). 
DShot is has discretised resolution, where as PWM was analogue on FC side (albeit discretised on the ESC due to hw limitations), 
however the advantages in accuracy, precision, resolution and communication outweigh this.

A Dshot *command* is constructed as follows:
- 11 bit number representing the *code* (0 to 2047)
- 1 bit telemetry flag
- 4 bit CRC

To transmit this *command* we construct a *frame* by converting the bits to a voltage signal. Each bit is represented as a duty cycle on pwm hw
- 0 = \< 33%
- 1 = > 75%
- Note that a prefix and suffix 0 duty cycle (not a 0 bit!) is required in transmission to tell each frame apart

The period of each bit (i.e. the period set in the PWM hw) is determined by by the DShot freq.
i.e. DShot150 sets the PWM hw freq to 150 kHz

Dshot *Codes*:
0 - Disarm (though not implemented?)
1 - 47: reserved for special use
48 - 2047: Throttle (2000 steps of precision)

For example:

Command: 1, Tel: 1
0x0033
Transmitted from left to right (I think)
LLLL LLLL LLHH LLHH

(Please note that this nomenclature may not be standard, but I believe it is clear)

---
## Thrust Test Stand

Aim: Gather data on motor performance

- linear ramp test
- step response
- code a pid loop, and see it's effect on motor performance


- Interestingly, since the number of dshot values is discretised, 
it might be possible to create a table characterising the response 
from one throttle value to another!?

The ESC needs commands to be sent within a set interval (currently undetermined). In order to satisfy this, the code is modelled as follows:

Timer raises interrupt at set-interval rate
    This will restart DMA (i.e. re-send last known command)
    OR a safety measure can be done to check if the system is responding, else send a disarm cmd / NOT send a cmd so that ESC disarms by itself

Main loop:
    Calculate next command to send
    Reset Timer interrupt
    Wait for current command to finish
    Send command

Debugging:
    Measure how many times interrupt is called
    Measure how long each main loop takes
    Measure minimum time before motor sends command

Additions:
    Look at using a two buffers. One to store the current command being executed. The second one should store the next command to write. The idea is that this elimantes the  overhead in copying from the temporary buffer to the main buffer; instead, we are swapping the reference to the buffer where DMA gets data from. however, this complicated things due to having to keep track of which buffer is which..

NOTE: 32 bit time is used
- `timer_hw->timerawl` returns a 32 bit number representing the number of microseconds since power on
- This will overflow at around 72 hrs
- We assume that this is longer than the operation time before the microcontroller is restarted
- NOTE that this has been the cause of many accidental failures in the past..

---
## Functions

- :tick: calculating duty cycle of bits from dshot speed
- enum to represent codes
- function in the processor to handle telemtry. Can this be handled in HW?
- code to packet (i.e. calculate checksum)

- Write some tests for these function!

NOTE:
- When re-setting DMA configuration, I tried just resetting the read address (assuming that the remaining configuration would stay the same). This did not work; one must re-configure the channel on every transfer. This seems inefficient.

---
## Sources

### USB Driver
- Use [Zadig](https://zadig.akeo.ie/) to install drivers for the RPi boot interface. This makes the flashing experience a LOT better! A thread on the [platform io forum](https://community.platformio.org/t/official-platformio-arduino-ide-support-for-the-raspberry-pi-pico-is-now-available/20792/9) goes in to more detail. [This thread[(https://community.platformio.org/t/raspberry-pi-pico-upload-problem/22809/7) also linked to this [github comment](https://github.com/platformio/platform-raspberrypi/issues/2#issuecomment-828586398) mentions to use Zadig. 

### Docs and Sample Implementation
- [Pico SDK API Docs](https://raspberrypi.github.io/pico-sdk-doxygen/modules.html). Some quick links: [dma](https://raspberrypi.github.io/pico-sdk-doxygen/group__hardware__dma)
- [Documentation on the Pico](https://www.raspberrypi.com/documentation/microcontrollers/?version=E0C9125B0D9B) incl spec, datasheets, [pinout](https://datasheets.raspberrypi.com/pico/Pico-R3-A4-Pinout.pdf), etc.
- [Pico examples](https://github.com/raspberrypi/pico-examples) from the rpi github incl `dma/`. There's an interesting example on pairing an adc with dma [here](https://github.com/raspberrypi/pico-examples/blob/master/adc/dma_capture/dma_capture.c). Note that when viewing pico examples, they use `#include "pico/stdlib.h"`. This is *not* to be used in the *Arduino* framework! as explained in [this post](https://community.platformio.org/t/include-pico-stdlib-h-causes-errors/22997). 

- [PlatformIO Documentation on Pico](https://docs.platformio.org/en/stable/boards/raspberrypi/pico.html#board-raspberrypi-pico). It only mentions the Arduino framework, but more seem to be avaialable (see other links here). 

### Explanations of DShot

- [Spencer's HW Blog](https://www.swallenhardware.io/battlebots/2019/4/20/a-developers-guide-to-dshot-escs) has a quick overview on the DShot protocol, list of the dshot command codes (which shd be sourced somewhere in the [betaflight repo](https://github.com/betaflight/betaflight)), and implmenetation overviews using scp and dma. 
- [This post](https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/) has a simple explanation of dshot with a few examples
- [DShot - the missing handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/) has supported hw, dshot frame example, arming, telemetry, bi-directional dshot
- [BLHeli dshot special command spec](https://github.com/bitdump/BLHeli/blob/master/BLHeli_32%20ARM/BLHeli_32%20Firmware%20specs/Digital_Cmd_Spec.txt)
- [Missing Handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/#special-commands) also has a good explanation of commands

### Other

- [Wiz IO Pico](https://github.com/Wiz-IO/wizio-pico): seems like an alternative to the Arduino framework used in PlatformIO? More details can be found on their [Baremetal wiki](https://github.com/Wiz-IO/wizio-pico/wiki/BAREMETAL)
- [Rpi Pico Forum post](https://forums.raspberrypi.com/viewtopic.php?t=332483). This person has balls to try and implemenet dshot using assembly!!

- List of [Arduino libraries for the RP2040](https://www.arduinolibraries.info/architectures/rp2040). I've not used any yet, but it might be best to use them as reference instead of importing them?

- [Upload port required issue](https://github.com/platformio/platform-raspberrypi/issues/2). I don't think this issue will be faced if using Zadig

