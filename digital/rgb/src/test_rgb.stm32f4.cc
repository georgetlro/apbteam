// rgb - Drives a sensors on an STM32F4 card. {{{
//
//
// Copyright (C) 2013 Maxime Hadjinlian
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
#include "rgb.stm32f4.hh"

ucoo::UsbStream usb_setup(){
    static ucoo::UsbStreamControl usc ("APBTeam", "USB test");
    static ucoo::UsbStream u = ucoo::UsbStream (usc, 0);
    // Don't block when writing
    u.block(false);
    return u;
}

void usb_write(ucoo::UsbStream u, char* value)
{
    char output[64];
    int len = 0;
    len = snprintf(output, sizeof(output), "%s\n", value);
    ucoo::delay_ms(5);
    u.write(output, len);
}

void
setup_input_capture()
{
    // Enable clock for Timer 2.
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);

    // The button is mapped to TIM2_CH1
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO5);

    // Enable interrupts for TIM2.
    nvic_enable_irq(NVIC_TIM2_IRQ);

    timer_set_mode(TIM2,
                   TIM_CR1_CKD_CK_INT, // Internal 72 MHz clock
                   TIM_CR1_CMS_EDGE,   // Edge synchronization
                   TIM_CR1_DIR_UP);    // Upward counter

    timer_set_prescaler(TIM2, 72-1);  // Counter unit = 1 us.
    timer_set_period(TIM2, 0xFFFF);
    timer_set_repetition_counter(TIM2, 0);
    timer_continuous_mode(TIM2);

    // Configure channel 1
    timer_ic_set_input(TIM2, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_filter(TIM2, TIM_IC1, TIM_IC_OFF);
    timer_ic_set_polarity(TIM2, TIM_IC1, TIM_IC_RISING);
    timer_ic_set_prescaler(TIM2, TIM_IC1, TIM_IC_PSC_OFF);
    timer_ic_enable(TIM2, TIM_IC1);
    timer_clear_flag(TIM2, TIM_SR_CC1IF);
    timer_enable_irq(TIM2, TIM_DIER_CC1IE);

    timer_enable_counter(TIM2);

}

void
setup_color(int new_color)
{
    ucoo::Gpio S2 (GPIOE, 2);
    ucoo::Gpio S3 (GPIOE, 3);
    S2.output();
    S3.output();
    if (new_color == RED)
    {
        S2.set(0);
        S3.set(0);
    }
    else if (new_color == GREEN)
    {
        S2.set(1);
        S3.set(1);
    }
    else if (new_color == BLUE)
    {
        S2.set(0);
        S3.set(1);
    }
    else if (new_color == WHITE)
    {
        S2.set(1);
        S3.set(0);
    }
}

// enable = 0 => enable the sensor, 1 disable
void
sensor_setup(int enable)
{
    ucoo::Gpio EN (GPIOE, 6);
    EN.output();
    EN.set(enable);
}

void
tim2_isr(void)
{
    static u32 last_value = 0;
    static u32 counter_time = 0;
    static u32 time = 0;
    static u32 value = 0;
    ucoo::Gpio led6 (GPIOD, 15);
    char color_out[64];
    ucoo::UsbStream usb = usb_setup();
    led6.output();
    if (timer_get_flag(TIM2, TIM_SR_CC1IF) != 0) {
        // Interupt received
        timer_clear_flag(TIM2, TIM_SR_CC1IF);
        if (cur_color == UNKNOWN){
            // Disabling the sensor should stop the inteupt also.
            sensor_setup(1);
        }
        // We want to throw a measure before changing color
        // since we do a "loop" of all the colors, we skip one out of two
        // measures.
        if (measure_cnt == 0)
        {
            // Leave this measure alone
            counter_time = timer_get_counter(TIM2);
            results[cur_color] = 0;
            snprintf(color_out, sizeof(color_out), "\n%s => %f", clr[cur_color], results[cur_color] );
            usb_write(usb, color_out);
            measure_cnt++;
        }
        else if (measure_cnt%2 == 1)
        {
            // First measure that counts.
            last_value = TIM2_CCR1;
            measure_cnt++;
        }
        else if (measure_cnt%2 == 0)
        {
            // Get TIMER counter
            //
            // Get a results
            value = (TIM2_CCR1 - last_value);
            time = (timer_get_counter(TIM2) - counter_time);
            if (time < 0)
                time = time + 0xffff;
            // if (cur_color == BLUE) {
                // results[cur_color] += value * 0.8;
            // }else if (cur_color == RED){
                // results[cur_color] += value * 1.2;
            // }else{
                results[cur_color] += value;
            // }
            snprintf(color_out, sizeof(color_out), "%s =>  %ld (Total :%f)", clr[cur_color], value, results[cur_color] );
            usb_write(usb, color_out);
            snprintf(color_out, sizeof(color_out), "Time is %ld", time);
            usb_write(usb, color_out);
            // measure_cnt++;
        // }
        // Means we have done 3 measure
        // if (measure_cnt == 41)
        // {
            // Grab the last results and move on to the next color
            // results[cur_color] /= 20;
            // snprintf(color_out, sizeof(color_out), "%s =>  Moy : %f", clr[cur_color], results[cur_color] );
            // usb_write(usb, color_out);

            cur_color++;
            setup_color(cur_color);
            measure_cnt = 0;
            counter_time = 0;
        }
    }
}

int
main (int argc, const char **argv)
{
    ucoo::arch_init (argc, argv);
    rcc_peripheral_enable_clock (&RCC_AHB1ENR, RCC_AHB1ENR_IOPEEN | RCC_AHB1ENR_IOPAEN | RCC_AHB1ENR_IOPDEN);

    ucoo::Gpio button (GPIOA, 0);
    ucoo::Gpio EN (GPIOE, 6);
    int i, color = 0;
    int min_color = 999;
    char color_out[64];

    button.input();

    ucoo::UsbStream usb = usb_setup();

    while(1){
        sensor_setup(1);
        setup_input_capture();

        usb_write(usb, "READY");

        // Wait for button to be pushed
        while (!button.get());
        // Wait for button to be released
        while (button.get());

        usb_write(usb, "WAIT");

        usb_write(usb, "START");
        setup_color(WHITE);
        EN.set(0);

        // Wait for the captor to be disabled
        while (EN.get() == 0);
        usb_write(usb, "DONE");

        for(i = 0; i < 4; i++)
        {
            snprintf(color_out, sizeof(color_out), "%s is %f\n", clr[i], results[i]);
            usb_write(usb, color_out);
            if (i >= 1 && results[i] < min_color)
            // if (results[i] < min_color)
            {
                color = i;
                min_color = results[i];
            }
            results[i] = 0;
        }
        snprintf(color_out, sizeof(color_out), "I think the object is %s\n", clr[color]);
        usb_write(usb, color_out);

        // Toggle the led we think is correct for 2s
        // RESET the color to check
        cur_color = WHITE;
        min_color = 999;
        color = 0;

        usb_write(usb, "END");

    }

    return 0;
}
