BASE = ../../../ucoolib

TARGETS = host stm32f4
PROGS = apbirthday
apbirthday_SOURCES = main.cc robot.cc hardware.host.cc hardware.stm32.cc \
		     simu_report.host.cc zb_avrisp.stm32.cc \
		     i2c_queue.cc asserv.cc mimot.cc lcd.cc beacon.cc \
		     potentiometer.host.cc potentiometer.stm32.cc \
		     pressure.cc chrono.host.cc chrono.stm32.cc debounce.cc servo.stm32.cc servo.host.cc\
		     rgb.stm32.cc rgb.host.cc \
		     radar.cc radar_2013.cc obstacles.cc path.cc path_2013.cc strat.cc \
		     outputs.cc \
		     top.cc init.cc move.cc candles.cc cannon.cc drinks.cc plate.cc \
		     angfsm.host.c angfsm_gen_arm_AI.arm.c fsm_queue.cc \
		     $(AVR_SOURCES)

MODULES = utils hal/gpio hal/adc hal/uart hal/usb hal/i2c hal/spi \
	  base/proto dev/avrisp dev/usdist

# Hack mode, include old code from AVR modules.
AVR_MODULES = math/geometry math/fixed path/astar
AVR_BASE = $(BASE)/../avr
INCLUDES += -I$(AVR_BASE) -I$(AVR_BASE)/common
host_DEFS += -DHOST
include $(AVR_MODULES:%=$(AVR_BASE)/modules/%/Makefile.module)
vpath %.c $(AVR_MODULES:%=$(AVR_BASE)/modules/%)
AVR_SOURCES = $(foreach module,$(AVR_MODULES),$($(subst /,_,$(module))_SOURCES))

# FSM.
INCLUDES += -I$(BASE)/../ai/src/fsm -I.
FSM_EMBEDDED_STRINGS = n
ifeq ($(FSM_EMBEDDED_STRINGS),y)
 ANGFSM_OPTIONS += --ang-embedded-strings
 DEFS += -DFSM_EMBEDDED_STRINGS=1
endif
vpath angfsm.host.c $(BASE)/../ai/src/fsm

# Common files.
INCLUDES += -I../common-cc
vpath %.cc ../common-cc

include $(BASE)/build/top.mk

clean: clean.project
clean.project:
	rm -f usb zb_uart angfsm_gen_arm_AI.h angfsm_gen_arm_AI.arm.c

obj/main.stm32f4.o: angfsm_gen_arm_AI.h
angfsm_gen_arm_AI.arm.c: angfsm_gen_arm_AI.h
angfsm_gen_arm_AI.h: apbirthday.host
	./$< $(ANGFSM_OPTIONS) --ang-gen arm && mv angfsm_gen_arm_AI.c angfsm_gen_arm_AI.arm.c

host: all.host
