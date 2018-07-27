#
# embedXcode
# ----------------------------------
# Embedded Computing on Xcode 4
#
# by corbin

# References and contribution
# ----------------------------------
# See About folder
# 


# Arduino 1.0 specifics for teensy3. not tested with anything else
# ----------------------------------
#
PLATFORM         := Arduino 
PLATFORM_TAG     := ARDUINO=10606
APPLICATION_PATH := /Applications/Arduino.app/Contents/Java

# avr for other chips.. #corbin
ARCHITECTURE := arm-none-eabi

# corbin, hardcoded for teensy again
APP_TOOLS_PATH   := $(APPLICATION_PATH)/hardware/tools/arm/bin
CORE_LIB_PATH    := $(APPLICATION_PATH)/hardware/teensy/avr/cores/teensy3
APP_LIB_PATH     := $(APPLICATION_PATH)/hardware/teensy/avr/libraries
BOARDS_TXT       := $(APPLICATION_PATH)/hardware/teensy/avr/boards.txt

# Sketchbook/Libraries path
# wildcard required for ~ management
#
ifeq ($(USER_PATH)/Library/Arduino/preferences.txt,)
    $(error Error: run Arduino once and define sketchbook path)
endif

ifeq ($(wildcard $(SKETCHBOOK_DIR)),)
    SKETCHBOOK_DIR = $(shell grep sketchbook.path $(USER_PATH)/Library/Arduino/preferences.txt | cut -d = -f 2)
endif
ifeq ($(wildcard $(SKETCHBOOK_DIR)),)
   $(error Error: sketchbook path not found)
endif
USER_LIB_PATH  = $(wildcard $(SKETCHBOOK_DIR)/Libraries)

# Rules for making a c++ file from the main sketch (.pde)
#
PDEHEADER      = \\\#include \"Arduino.h\"  

# Tool-chain names
#

# hack, corbin..hardcoded path...for now. mk20dx256
CC      = $(APP_TOOLS_PATH)/$(ARCHITECTURE)-gcc
CXX     = $(APP_TOOLS_PATH)/$(ARCHITECTURE)-g++
# don't AR for Teensy 3.1
AR      =
OBJDUMP = $(APP_TOOLS_PATH)/$(ARCHITECTURE)-objdump
OBJCOPY = $(APP_TOOLS_PATH)/$(ARCHITECTURE)-objcopy
SIZE    = $(APP_TOOLS_PATH)/$(ARCHITECTURE)-size
NM      = $(APP_TOOLS_PATH)/$(ARCHITECTURE)-nm

# Specific AVRDUDE location and options
#
#AVRDUDE_COM_OPTS  = -D -p$(MCU) -C$(AVRDUDE_CONF) # corbin

BOARD    = $(call PARSE_BOARD,$(BOARD_TAG),board)
#LDSCRIPT = $(call PARSE_BOARD,$(BOARD_TAG),ldscript)
#VARIANT  = $(call PARSE_BOARD,$(BOARD_TAG),build.variant) # could be dynamic, but needs testing
VARIANT_PATH = $(APPLICATION_PATH)/hardware/teensy/teensy3


MCU_FLAG_NAME  = mcpu
#corbin, hacked..hardcoded
MCU = cortex-m4
#$(call PARSE_BOARD,$(BOARD_TAG),build.mcu)
# not build.mcu, which is what Step2.mk does by default



EXTRA_LDFLAGS = -larm_cortexM4l_math -T$(CORE_LIB_PATH)/mk20dx256.ld -mthumb

#TODO: parse these! Every time i update teensduiino i'm killinmyself...
#teensy31.build.option1=-mthumb
#teensy31.build.option2=-nostdlib
#teensy31.build.option3=-D__MK20DX256__
#teensy31.build.option4=-DTEENSYDUINO=126

EXTRA_CPPFLAGS = -MMD -DUSB_VID=null -DUSB_PID=null -I$(VARIANT_PATH) -D$(PLATFORM_TAG) -mthumb  -D__MK20DX256__ -DUSB_SERIAL -DLAYOUT_US_ENGLISH -DTEENSYDUINO=126 -DFASTLED_TEENSY3=1


# Leonardo USB PID VID
#
# corbin, hook these up in stead of hard coding?
USB_VID := $(call PARSE_BOARD,$(BOARD_TAG),build.vid)
USB_PID := $(call PARSE_BOARD,$(BOARD_TAG),build.pid)

ifneq ($(USB_PID),)
ifneq ($(USB_VID),)
    USB_FLAGS  = -DUSB_VID=$(USB_VID)
    USB_FLAGS += -DUSB_PID=$(USB_PID)
    USB_RESET  = $(UTILITIES_PATH)/serial1200.py
endif
endif



# corbin, now hardcoded; this moved to platform.txt, and I don't parse it..
# $(call PARSE_BOARD,$(BOARD_TAG),build.post_compile_script)
POSTCOMPILE_COMMAND_SCRIPT_NAME = teensy_post_compile
POSTCOMPILE_COMMAND = $(APPLICATION_PATH)/hardware/tools/$(POSTCOMPILE_COMMAND_SCRIPT_NAME) -tools=$(APPLICATION_PATH)/hardware/tools/

#corbin, again hardcoded
#$(call PARSE_BOARD,$(BOARD_TAG),upload.avrdude_wrapper)
REBOOT_COMMAND_SCRIPT_NAME = teensy_reboot
REBOOT_COMMAND = $(APPLICATION_PATH)/hardware/tools/$(REBOOT_COMMAND_SCRIPT_NAME)



