BASE = ../../avr
AVR_PROGS = test_rvb
test_rvb_SOURCES = test_rvb.c
MODULES = proto uart utils
CONFIGFILE = avrconfig.h
# atmega8, atmega8535, atmega128...
AVR_MCU = atmega64
# -O2 : speed
# -Os : size
OPTIMIZE = -O2

include $(BASE)/make/Makefile.gen
