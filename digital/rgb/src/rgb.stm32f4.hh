#include "ucoolib/arch/arch.hh"
#include "ucoolib/hal/gpio/gpio.hh"
#include "ucoolib/hal/usb/usb.hh"
#include "ucoolib/utils/delay.hh"
#include "ucoolib/common.hh"

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <cstdio>

static enum colors{
    WHITE = 0,
    RED = 1,
    GREEN = 2,
    BLUE = 3,
    UNKNOWN = 4
} color;
const char * clr[] = { "WHITE", "RED", "GREEN", "BLUE", "UNK"};

int setup_measure ();
void tim2_isr(void);
void sensor_setup(int enable);
void setup_color(int new_color);
void setup_input_capture();

