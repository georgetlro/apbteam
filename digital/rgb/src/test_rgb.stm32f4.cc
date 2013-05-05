#include "rgb.stm32f4.hh"

#define BASIC_GREY 190

static int measure_cnt = 0;
static int cur_color = WHITE;
static float results[4] = {0, 0, 0, 0};

ucoo::UsbStream usb_setup(){
    static ucoo::UsbStreamControl usc ("APBTeam", "USB test");
    static ucoo::UsbStream u = ucoo::UsbStream (usc, 0);
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

        gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO5);

        nvic_enable_irq(NVIC_TIM2_IRQ);

   timer_set_mode(TIM2,
                   TIM_CR1_CKD_CK_INT, // Internal 72 MHz clock
                   TIM_CR1_CMS_EDGE,   // Edge synchronization
                   TIM_CR1_DIR_UP);    // Upward counter

    timer_set_prescaler(TIM2, 75);
    timer_set_period(TIM2, 0xFFFF);
    timer_set_repetition_counter(TIM2, 0);
    timer_continuous_mode(TIM2);

    // Configure channel 1
    timer_ic_set_input(TIM2, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_filter(TIM2, TIM_IC1, TIM_IC_OFF);
    timer_ic_set_polarity(TIM2, TIM_IC1, TIM_IC_RISING);
    // timer_ic_set_prescaler(TIM2, TIM_IC1, TIM_IC_PSC_8);
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

static u32 last_value = 0;
void
tim2_isr(void)
{
    static u32 value = 0;
    char color_out[64];
    cur_color = WHITE;
    if (timer_get_flag(TIM2, TIM_SR_CC1IF) != 0) {
        timer_clear_flag(TIM2, TIM_SR_CC1IF);
        if (measure_cnt == 0)
        {
            results[cur_color] = 0;
            measure_cnt++;
        }
        else if (measure_cnt == 1)
        {
            last_value = TIM2_CCR1;
            measure_cnt++;
        }
        else if (measure_cnt == 2)
        {
            if (TIM2_CCR1 > last_value)
            {
                results[cur_color] = (TIM2_CCR1 - last_value);
            }
            else
            {
                results[cur_color] = (65535 - last_value) + TIM2_CCR1;
            }
            // cur_color++;
            // if (cur_color == UNKNOWN)
            // {
                // cur_color = WHITE;
                sensor_setup(1);
            // }
            // setup_color(cur_color);
            measure_cnt = 0;
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

    int i = 0, color = 0;
    int min_color = 999;
    uint32_t sumup_color = 0;
    uint32_t offset = 0;
    int nb_cherry = 0;
    char color_out[64];

    button.input();

    ucoo::UsbStream usb = usb_setup();

    sensor_setup(1);
    setup_input_capture();
    // setup_color(WHITE);
    setup_color(BLUE);
    // setup_color(RED);

    ucoo::Gpio led3 (GPIOD, 13);
    led3.output();
    led3.reset();

    // Grab a first measure to compute the offet from the "normal" grey
    sensor_setup(0);
    while (EN.get() == 0);
    offset = BASIC_GREY - results[0];

    while(1){

        sensor_setup(0);
        while (EN.get() == 0);

        // snprintf(color_out, sizeof(color_out), "%f", results[0]);
        // usb_write(usb, color_out);
        color = results[0] + offset;

        if (color > 120 && color < 165)
        {
            // usb.write("W", 1);
            ++nb_cherry;
            snprintf(color_out, sizeof(color_out), "C-%f-%d", results[0], nb_cherry);
            usb_write(usb, color_out);
        }
        else if (color > 40 && color < 100)
        {
            // usb.write("W", 1);
            ++nb_cherry;
            snprintf(color_out, sizeof(color_out), "W-%f-%d", results[0], nb_cherry);
            usb_write(usb, color_out);
        }
        else
        {
            ucoo::delay_ms(1);
        }

        // if (results[0] < 600)
        // {
        //     // usb.write("W", 1);
        //     snprintf(color_out, sizeof(color_out), "W-%f", results[0]);
        //     usb_write(usb, color_out);
        // }
        // else if (results[0] > 1200)
        // {
        //     // usb.write("N", 1);
        //     ucoo::delay_ms(1);
        // }
        // else
        // {
        //     // usb.write("C", 1);
        //     snprintf(color_out, sizeof(color_out), "C-%f", results[0]);
        //     usb_write(usb, color_out);
        // }

        i++;
        if ( i >= 50)
        {
            led3.toggle();
            i = 0;
        }
        ucoo::delay_ms(3);
    }

    return 0;
}
