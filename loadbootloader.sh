#!/bin/sh

#-f "/Users/pete/samd21/9DOF_Razor_IMU/build/src/hardware/samd/1.3.2/variants/arduino_zero/openocd_scripts/arduino_zero.cfg" \

openocd -d2 -s "/Applications/GNU ARM Eclipse/OpenOCD/0.10.0-201701241841/bin/openocd" \
  -f "/Users/pete/samd21/atmel_samd21e18a-jlink.cfg" \
  -c "telnet_port disabled; init; halt; at91samd bootloader 0; program {{/Users/pete/samd21/arduino-samd-bootloader/arduino-1.6.15-bootloaders/zero/samd21_sam_ba.bin}} verify reset; shutdown"
