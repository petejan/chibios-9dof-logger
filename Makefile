##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O1 -ggdb -fomit-frame-pointer -falign-functions=16 --specs=nano.specs
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = -Wno-unused-parameter
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = -lm
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x400
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

# Enables the use of FPU (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = no
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = ch

ASF_PATH = ../../xdk-asf-3.34.2

# Imported source files and paths
CHIBIOS = ../../ChibiOS_17.6.0
# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_samd21.mk
# HAL-OSAL files (optional).
#include $(CHIBIOS)/os/hal/hal.mk
#include $(CHIBIOS)/os/hal/ports/STM32/STM32F0xx/platform.mk
#include $(CHIBIOS)/os/hal/boards/ST_STM32F072B_DISCOVERY/board.mk
#include $(CHIBIOS)/os/hal/osal/nil/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/nil/nil.mk
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v6m.mk
# Other files (optional).
#include $(CHIBIOS)/test/nil/test.mk

# Define linker script file here
LDSCRIPT= $(STARTUPLD)/samd21g18au_flashx.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(STARTUPSRC) \
       $(KERNSRC) \
       $(PORTSRC) \
       $(OSALSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(TESTSRC) \
       $(ASF_PATH)/sam0/drivers/sercom/sercom.c \
       $(ASF_PATH)/sam0/drivers/sercom/usart/usart.c \
       $(ASF_PATH)/sam0/drivers/sercom/spi/spi.c \
       $(ASF_PATH)/sam0/drivers/sercom/i2c/i2c_samd20/i2c_master.c \
       $(ASF_PATH)/sam0/drivers/system/clock/clock_samd21_r21_da_ha1/clock.c \
       $(ASF_PATH)/sam0/drivers/system/clock/clock_samd21_r21_da_ha1/gclk.c \
       $(ASF_PATH)/sam0/drivers/rtc/rtc_sam_d_r_h/rtc_calendar.c \
       $(ASF_PATH)/common/utils/interrupt/interrupt_sam_nvic.c \
       $(ASF_PATH)/sam0/drivers/system/pinmux/pinmux.c \
       $(ASF_PATH)/sam0/utils/syscalls/gcc/syscalls.c \
       $(ASF_PATH)/sam0/drivers/system/system.c \
       $(ASF_PATH)/sam0/drivers/port/port.c \
       $(ASF_PATH)/sam0/drivers/dma/dma.c \
       $(ASF_PATH)/sam0/drivers/extint/extint_callback.c \
       $(ASF_PATH)/sam0/drivers/extint/extint_sam_d_r_h/extint.c  \
       src/xprintf.c \
       src/util/inv_mpu.c \
       src/util/inv_mpu_dmp_motion_driver.c \
       src/diskio.c \
       src/usb/sam_ba_cdc.c \
       src/usb/sam_ba_usb.c \
       src/usb/board_driver_usb.c \
       src/adcThread.c \
       src/fatfs/source/ff.c \
       src/fatfs/source/ffsystem.c \
       src/fatfs/source/ffunicode.c


#       $(ASF_PATH)/sam0/drivers/sercom/usart/usart_interrupt.c \
#       $(ASF_PATH)/sam0/drivers/sercom/sercom.c \
#       $(ASF_PATH)/sam0/drivers/sercom/sercom_interrupt.c \
       
# List of C source files.¬
#CSRCS = \¬
#       common/utils/interrupt/interrupt_sam_nvic.c         \¬
#       ../usart/src/boards/9DOF-RAZOR/board_init.c         \¬
#       sam0/drivers/port/port.c                            \¬
#       sam0/drivers/sercom/sercom.c                        \¬
#       ../usart/src/qs_usart_basic_use.c                   \¬
#       sam0/drivers/sercom/usart/usart.c                   \¬
#       sam0/drivers/system/clock/clock_samd21_r21_da_ha1/clock.c \¬
#       sam0/drivers/system/clock/clock_samd21_r21_da_ha1/gclk.c  \¬
#       sam0/drivers/system/interrupt/system_interrupt.c    \¬
#       sam0/drivers/system/pinmux/pinmux.c                 \¬
#       sam0/drivers/system/system.c                        \¬
#       sam0/utils/cmsis/samd21/source/gcc/startup_samd21.c \¬
#       sam0/utils/cmsis/samd21/source/system_samd21.c      \¬
#       sam0/utils/syscalls/gcc/syscalls.c¬

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = \
       src/TinyGPSPlus-0.95/TinyGPS++.cpp \
       src/MadgwickAHRS.cpp \
       src/usbThread.cpp \
       src/gpsThread.cpp \
       src/mpuThread.cpp \
       src/main.cpp

#        src/fxThread.cpp \


# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC =
ASMXSRC = $(STARTUPASM) $(PORTASM) $(OSALASM)

INCDIR = $(CHIBIOS)/os/license \
         $(STARTUPINC) $(KERNINC) $(PORTINC) $(OSALINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) $(TESTINC) \
         src \
         src/usb \
		 $(ASF_PATH)/sam0/utils \
		 $(ASF_PATH)/sam0/utils/preprocessor \
		 $(ASF_PATH)/sam0/utils/header_files \
		 $(ASF_PATH)/sam0/utils/cmsis/samd21/include \
		 $(ASF_PATH)/sam0/utils/cmsis/samd21/source \
		 $(ASF_PATH)/sam0/drivers/port \
		 $(ASF_PATH)/sam0/drivers/sercom \
		 $(ASF_PATH)/sam0/drivers/sercom/usart \
		 $(ASF_PATH)/sam0/drivers/sercom/i2c \
		 $(ASF_PATH)/sam0/drivers/sercom/i2c/i2c_sam0 \
		 $(ASF_PATH)/sam0/drivers/sercom/spi \
		 $(ASF_PATH)/sam0/drivers/rtc \
		 $(ASF_PATH)/sam0/drivers/extint \
		 $(ASF_PATH)/sam0/drivers/system \
		 $(ASF_PATH)/sam0/drivers/system/clock \
		 $(ASF_PATH)/sam0/drivers/system/clock/clock_samd21_r21_da_ha1 \
		 $(ASF_PATH)/sam0/drivers/system/power \
		 $(ASF_PATH)/sam0/drivers/system/power/power_sam_d_r_h \
		 $(ASF_PATH)/sam0/drivers/system/reset \
		 $(ASF_PATH)/sam0/drivers/system/reset/reset_sam_d_r_h \
		 $(ASF_PATH)/sam0/drivers/system/interrupt \
		 $(ASF_PATH)/sam0/drivers/system/interrupt/system_interrupt_samd21 \
		 $(ASF_PATH)/sam0/drivers/system/pinmux \
		 $(ASF_PATH)/sam0/drivers/dma \
		 $(ASF_PATH)/common/utils \
		 $(ASF_PATH)/common/boards \
         $(CHIBIOS)/os/various 


#       common/boards                                      \¬
#       common/utils                                       \¬
#       sam0/boards                                        \¬
#       sam0/drivers/port                                  \¬
#       sam0/drivers/sercom                                \¬
#       sam0/drivers/sercom/usart                          \¬
#       sam0/drivers/sercom/usart/quick_start              \¬
#       sam0/drivers/sercom/usart/quick_start/samd21_xplained_pro \¬
#       sam0/drivers/system                                \¬
#       sam0/drivers/system/clock                          \¬
#       sam0/drivers/system/clock/clock_samd21_r21_da_ha1  \¬
#       sam0/drivers/system/interrupt                      \¬
#       sam0/drivers/system/interrupt/system_interrupt_samd21 \¬
#       sam0/drivers/system/pinmux                         \¬
#       sam0/drivers/system/power                          \¬
#       sam0/drivers/system/power/power_sam_d_r_h          \¬
#       sam0/drivers/system/reset                          \¬
#       sam0/drivers/system/reset/reset_sam_d_r_h          \¬
#       sam0/utils/cmsis/samd21/include                    \¬
#       sam0/utils/cmsis/samd21/source                     \¬
#       sam0/utils/header_files                            \¬
#       sam0/utils/preprocessor                            \¬
#       thirdparty/CMSIS/Include                           \¬
#       thirdparty/CMSIS/Lib/GCC                           \¬
#       ../usart/src¬
#       ../usart/src/boards/9DOF-RAZOR                     \¬

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m0plus

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra -Wundef

#
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = -D __SAMD21G18A__ -D __samd21j18a__ \
        -D BOARD=USER_BOARD -D USART_CALLBACK_MODE=false -D ARM_MATH_CM0PLUS=true -D I2C_MASTER_CALLBACK_MODE=false \
        -D RTC_CALENDAR_ASYNC=false \
        -D EXTINT_CALLBACK_MODE=true \
        -D SPI_CALLBACK_MODE=false \
        -D printf=iprintf \
        -D BOARD_ID_sparkfun_9dof \
        -D CORTEX_ALTERNATE_SWITCH=TRUE

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR = $(ASF_PATH)/thirdparty/CMSIS/Lib/GCC

# List all user libraries here
ULIBS = -larm_cortexM0l_math

#
# End of user defines
##############################################################################

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC
include $(RULESPATH)/rules.mk

##############################################################################
# MISRA check rule, requires PCLint and the setup files, not provided.
#
misra:
	@lint-nt -v -w3 $(DEFS) pclint/co-gcc.lnt pclint/au-misra3.lnt pclint/waivers.lnt $(IINCDIR) $(CSRC) &> misra.txt
