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
#include "hardware.hh"

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>
#include "ucoolib/hal/gpio/gpio.hh"

#include "zb_avrisp.stm32.hh"

Hardware::Hardware ()
    : dev_uart (4), zb_uart (2),
      usb_control ("APBTeam", "APBirthday"), usb (usb_control, 0),
      zb_usb_avrisp (usb_control, 1),
      main_i2c (2), zb_i2c (1),
      raw_jack (GPIOD, 12),
      ihm_color (GPIOD, 14),
      ihm_strat (GPIOD, 13),
      ihm_robot_nb (GPIOD, 15),
      ihm_lol (GPIOD, 11),
      ihm_emerg_stop (GPIOC, 6),
      glass_contact (GPIOE, 5),
      cherry_plate_left_contact (GPIOE, 6),
      cherry_plate_right_contact (GPIOC, 13),
      cake_arm_out_contact (GPIOC, 5), cake_arm_in_contact (GPIOB, 1),
      cherry_bad_out (GPIOE, 0), cherry_bad_in (GPIOE, 1),
      cherry_plate_up (GPIOE, 2), cherry_plate_down (GPIOE, 3),
      cherry_plate_clamp (GPIOE, 4),
      cake_rgb_far_en (GPIOE, 10),
      cake_arm_out (GPIOB, 3), cake_arm_in (GPIOB, 4),
      cake_push_far_out (GPIOD, 7), cake_push_far_in (GPIOB, 5),
      cake_push_near_out (GPIOD, 6), cake_push_near_in (GPIOB, 6),
      cannon_rgb_2 (GPIOE, 12),
      glass_lower_clamp_close (GPIOA, 15), glass_lower_clamp_open (GPIOA, 10),
      glass_upper_clamp_close (GPIOD, 0), glass_upper_clamp_open (GPIOD, 1),
      glass_upper_clamp_up (GPIOD, 3), glass_upper_clamp_down (GPIOC, 10),
      gift_out (GPIOD, 4), gift_in (GPIOC, 11),
      ballon_funny_action (GPIOB, 7),
      pneum_open (GPIOD, 5),
      dist0_sync (GPIOC, 0), dist1_sync (GPIOC, 1),
      dist2_sync (GPIOC, 2), dist3_sync (GPIOC, 3),
      adc (0),
      adc_dist0 (adc, 0), adc_dist1 (adc, 1),
      adc_dist2 (adc, 2), adc_dist3 (adc, 3),
      adc_cake_front (adc, 6), adc_cake_back (adc, 7),
      adc_pressure (adc, 8)
{
    rcc_peripheral_enable_clock (&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
    rcc_peripheral_enable_clock (&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
    rcc_peripheral_enable_clock (&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);
    rcc_peripheral_enable_clock (&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
    rcc_peripheral_enable_clock (&RCC_AHB1ENR, RCC_AHB1ENR_IOPEEN);
    // dev_uart
    gpio_mode_setup (GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12);
    gpio_mode_setup (GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
    gpio_set_af (GPIOC, GPIO_AF8, GPIO12);
    gpio_set_af (GPIOD, GPIO_AF8, GPIO2);
    dev_uart.enable (38400, ucoo::Uart::EVEN, 1);
    dev_uart.block (false);
    // zb_uart
    gpio_mode_setup (GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
    gpio_set_af (GPIOD, GPIO_AF7, GPIO8 | GPIO9);
    zb_uart.enable (38400, ucoo::Uart::EVEN, 1);
    zb_uart.block (false);
    // usb
    usb.block (false);
    zb_usb_avrisp.block (false);
    // main_i2c
    gpio_mode_setup (GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
    gpio_mode_setup (GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_set_output_options (GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO8);
    gpio_set_output_options (GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO9);
    gpio_set_af (GPIOA, GPIO_AF4, GPIO8);
    gpio_set_af (GPIOC, GPIO_AF4, GPIO9);
    main_i2c.enable ();
    // zb_i2c
    gpio_mode_setup (GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11);
    gpio_set_output_options (GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ,
                             GPIO10 | GPIO11);
    gpio_set_af (GPIOB, GPIO_AF4, GPIO10 | GPIO11);
    zb_i2c.enable ();
    // ADC.
    gpio_mode_setup (GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
                     GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO6 | GPIO7);
    gpio_mode_setup (GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
    adc.enable ();
    // Cycle timer, 4 ms period.
    rcc_peripheral_enable_clock (&RCC_APB1ENR, RCC_APB1ENR_TIM3EN);
    TIM3_PSC = 2 * rcc_ppre1_frequency / 1000000 - 1; // 1 µs prescaler
    TIM3_ARR = 4000 - 1;
    TIM3_EGR = TIM_EGR_UG;
    TIM3_CR1 = TIM_CR1_CEN;
}

void
Hardware::wait ()
{
    while (!(TIM3_SR & TIM_SR_UIF))
        ;
    TIM3_SR = ~TIM_SR_UIF;
}

void
Hardware::zb_handle ()
{
    // Switch to AVRISP?
    if (zb_usb_avrisp.poll ())
        zb_handle (zb_usb_avrisp);
}

void
Hardware::zb_handle (ucoo::Stream &s)
{
    // Uart pins are reused.
    zb_uart.disable ();
    gpio_mode_setup (GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO8 | GPIO9);
    // Go to special AVRISP mode.
    zb_avrisp (s);
    // Restore.
    gpio_mode_setup (GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
    gpio_set_af (GPIOD, GPIO_AF7, GPIO8 | GPIO9);
    zb_uart.enable (38400, ucoo::Uart::EVEN, 1);
}

