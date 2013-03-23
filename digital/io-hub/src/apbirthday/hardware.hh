#ifndef hardware_hh
#define hardware_hh
// io-hub - Modular Input/Output. {{{
//
// Copyright (C) 2013 Nicolas Schodet
//
// APBTeam:
//        Web: http://apbteam.org/
//      Email: team AT apbteam DOT org
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//
// }}}

#include "ucoolib/hal/uart/uart.hh"
#include "ucoolib/hal/gpio/gpio.hh"
#include "ucoolib/hal/i2c/i2c.hh"
#ifdef TARGET_stm32
# include "ucoolib/hal/usb/usb.hh"
#else
# include "ucoolib/arch/host/host_stream.hh"
#endif

#ifdef TARGET_host
# include "ucoolib/arch/host/host.hh"

/// Class to work around C++ initializer list syntax.
class HardwareHost : public ucoo::Host
{
  public:
    HardwareHost ();
};

#endif

struct Hardware
{
#ifdef TARGET_host
    HardwareHost host;
#endif
    ucoo::Uart dev_uart, zb_uart;
#ifdef TARGET_stm32
    ucoo::UsbStreamControl usb_control;
    ucoo::UsbStream usb, zb_usb_avrisp;
    ucoo::I2cHard main_i2c;
#else
    ucoo::HostStream usb;
    ucoo::I2cHost main_i2c;
#endif
    ucoo::Gpio
        raw_jack,
        ihm_color,
        ihm_strat,
        ihm_robot_nb,
        ihm_lol,
        ihm_emerg_stop;
    ucoo::Gpio
        glass_contact,
        cherry_plate_left_contact, cherry_plate_right_contact;
    ucoo::Gpio
        cherry_bad_out, cherry_bad_in,
        cherry_plate_up, cherry_plate_down,
        cherry_plate_clamp,
        cake_arm_out, cake_arm_in,
        cake_push_far_out, cake_push_far_in,
        cake_push_near_out, cake_push_near_in,
        glass_lower_clamp_close, glass_lower_clamp_open,
        glass_upper_clamp_close, glass_upper_clamp_open,
        glass_upper_clamp_up, glass_upper_clamp_down,
        gift_out, gift_in,
        ballon_funny_action,
        pneum_open;
    static const int inputs_nb = 9;
    static const int outputs_nb = 21;
    Hardware ();
    // Wait until next cycle.
    void wait ();
    // Handle zb programmation.
    void zb_handle ();
};

#endif // hardware_hh
