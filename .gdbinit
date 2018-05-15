target remote localhost:3333
#target remote | openocd -f atmel_samd21e18a-jlink.cfg -c "gdb_port pipe; log_output openocd.log"

monitor reset init
monitor halt

monitor arm semihosting enable
symbol-file "build/ch.elf"
load "build/ch.elf"
tbreak main
monitor reset halt
monitor halt

#continue
