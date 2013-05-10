//  {{{
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
#include "lcd.hh"
#include "ucoolib/utils/delay.hh"
#include "font.hh"


LCD::LCD ()
    : cs_ (GPIOB, 10), rs_ (GPIOB, 11), wr_ (GPIOC, 9), rd_ (GPIOC, 6),
      reset_ (GPIOC, 7), bl_ (GPIOA, 8)
{
    // Setup GPIOs.
    cs_.set (); rs_.set (); wr_.set (); rd_.set (); reset_.set ();
    bl_.reset ();
    gpio_mode_setup (GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                     GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4
                     | GPIO5 | GPIO6 | GPIO7 | GPIO8);
    gpio_mode_setup (GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                     GPIO0 | GPIO1 | GPIO10 | GPIO11);
    gpio_mode_setup (GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                     GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4
                     | GPIO5 | GPIO6 | GPIO7 | GPIO9);
    gpio_set_output_options (GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                             GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4
                             | GPIO5 | GPIO6 | GPIO7 | GPIO8);
    gpio_set_output_options (GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                             GPIO0 | GPIO1 | GPIO10 | GPIO11);
    gpio_set_output_options (GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                             GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4
                             | GPIO5 | GPIO6 | GPIO7 | GPIO9);
    // Initialisation.
    //  Reset pulse.
    reset_.reset ();
    ucoo::delay_ms (100);
    reset_.set ();
    //  Read device code.
    uint16_t device_code = read_reg (0);
    ucoo::assert (device_code == 0x8989);
    //  Run manufacturer init code.
    write_reg (0x0000, 0x0001); ucoo::delay_ms (50);
    write_reg (0x0003, 0xa8a4); ucoo::delay_ms (50);
    write_reg (0x000c, 0x0000); ucoo::delay_ms (50);
    write_reg (0x000d, 0x080c); ucoo::delay_ms (50);
    write_reg (0x000e, 0x2b00); ucoo::delay_ms (50);
    write_reg (0x001e, 0x00b0); ucoo::delay_ms (50);
    write_reg (0x0001, 0x2b3f); ucoo::delay_ms (50);
    write_reg (0x0002, 0x0600); ucoo::delay_ms (50);
    write_reg (0x0010, 0x0000); ucoo::delay_ms (50);
    write_reg (0x0011, 0x6058); ucoo::delay_ms (50);
    write_reg (0x0005, 0x0000); ucoo::delay_ms (50);
    write_reg (0x0006, 0x0000); ucoo::delay_ms (50);
    write_reg (0x0016, 0xef1c); ucoo::delay_ms (50);
    write_reg (0x0017, 0x0003); ucoo::delay_ms (50);
    write_reg (0x0007, 0x0133); ucoo::delay_ms (50);
    write_reg (0x000b, 0x0000); ucoo::delay_ms (50);
    write_reg (0x000f, 0x0000); ucoo::delay_ms (50);
    write_reg (0x0041, 0x0000); ucoo::delay_ms (50);
    write_reg (0x0042, 0x0000); ucoo::delay_ms (50);
    write_reg (0x0048, 0x0000); ucoo::delay_ms (50);
    write_reg (0x0049, 0x013f); ucoo::delay_ms (50);
    write_reg (0x004a, 0x0000); ucoo::delay_ms (50);
    write_reg (0x004b, 0x0000); ucoo::delay_ms (50);
    write_reg (0x0044, 0xef00); ucoo::delay_ms (50);
    write_reg (0x0045, 0x0000); ucoo::delay_ms (50);
    write_reg (0x0046, 0x013f); ucoo::delay_ms (50);
    write_reg (0x0030, 0x0707); ucoo::delay_ms (50);
    write_reg (0x0031, 0x0204); ucoo::delay_ms (50);
    write_reg (0x0032, 0x0204); ucoo::delay_ms (50);
    write_reg (0x0033, 0x0502); ucoo::delay_ms (50);
    write_reg (0x0034, 0x0507); ucoo::delay_ms (50);
    write_reg (0x0035, 0x0204); ucoo::delay_ms (50);
    write_reg (0x0036, 0x0204); ucoo::delay_ms (50);
    write_reg (0x0037, 0x0502); ucoo::delay_ms (50);
    write_reg (0x003a, 0x0302); ucoo::delay_ms (50);
    write_reg (0x003b, 0x0302); ucoo::delay_ms (50);
    write_reg (0x0023, 0x0000); ucoo::delay_ms (50);
    write_reg (0x0024, 0x0000); ucoo::delay_ms (50);
    write_reg (0x0025, 0x8000); ucoo::delay_ms (50);
    write_reg (0x004f, 0);
    write_reg (0x004e, 0);
    // OK, display on.
    rainbow ();
    on ();
}

void
LCD::on ()
{
    bl_.set ();
}

void
LCD::off ()
{
    bl_.reset ();
}

void
LCD::clear (uint16_t color)
{
    set_cursor (0, 0);
    cs_.reset ();
    write_index (0x0022);
    for (int i = 0; i < x_max * y_max; i++)
        write_data (color);
    cs_.set ();
}

void
LCD::rainbow ()
{
    set_cursor (0, 0);
    cs_.reset ();
    write_index (0x0022);
    for (int y = 0; y < y_max; y++)
        for (int x = 0; x < x_max; x++)
        {
            int r = x * 0xff / (x_max - 1);
            int g = y * 0xff / (y_max - 1);
            int b = 0xff - r;
            write_data (color (r, g, b));
        }
    cs_.set ();
}


void
LCD::set_cursor (int x, int y)
{
    write_reg (0x004e, y);
    write_reg (0x004f, x_max - 1 - x);
}

void
LCD::write_index (uint16_t index)
{
    rs_.reset ();
    GPIOC_ODR = (GPIOC_ODR & ~0x3f) | (index & 0x3f);
    GPIOA_ODR = (GPIOA_ODR & ~0xff) | ((index >> 6) & 0xff);
    GPIOB_ODR = (GPIOB_ODR & ~0x3) | ((index >> 14) & 0x3);
    wr_.reset ();
    wr_.set ();
}

void
LCD::write_data (uint16_t data)
{
    rs_.set ();
    GPIOC_ODR = (GPIOC_ODR & ~0x3f) | (data & 0x3f);
    GPIOA_ODR = (GPIOA_ODR & ~0xff) | ((data >> 6) & 0xff);
    GPIOB_ODR = (GPIOB_ODR & ~0x3) | ((data >> 14) & 0x3);
    wr_.reset ();
    wr_.set ();
}

uint16_t
LCD::read_data ()
{
    uint16_t value;
    gpio_mode_setup (GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
                     GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4
                     | GPIO5 | GPIO6 | GPIO7);
    gpio_mode_setup (GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
                     GPIO0 | GPIO1);
    gpio_mode_setup (GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
                     GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4
                     | GPIO5);
    rs_.set ();
    rd_.reset ();
    ucoo::delay_us (1);
    value = (GPIOC_IDR & 0x3f)
        | ((GPIOA_IDR & 0xff) << 6)
        | ((GPIOB_IDR & 0x3) << 14);
    rd_.set ();
    gpio_mode_setup (GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                     GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4
                     | GPIO5 | GPIO6 | GPIO7);
    gpio_mode_setup (GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                     GPIO0 | GPIO1);
    gpio_mode_setup (GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                     GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4
                     | GPIO5);
    return value;
}

void
LCD::write_reg (uint16_t index, uint16_t data)
{
    cs_.reset ();
    write_index (index);
    write_data (data);
    cs_.set ();
}

uint16_t
LCD::read_reg (uint16_t index)
{
    uint16_t data;
    cs_.reset ();
    write_index (index);
    data = read_data ();
    cs_.set ();
    return data;
}
void 
LCD::rectangle_fill (uint16_t color,Rect format, Rect pos)//make a filled rectangle 
{
	set_cursor(pos.x , pos.y);
	if(format.x+pos.x> x_max){format.x=x_max-pos.x;}
	if(format.y+pos.y> y_max){format.y=y_max-pos.y;}
	if(format.x>=0 && format.y >=0)
	{	
	for(int i=0 ; i<format.y ; i++)
	{
		cs_.reset();
		write_index (0x0022);
		for(int j=0 ; j<format.x ; j++)
		{
			write_data(color);
		}
		cs_.set();
	pos.y++;
	set_cursor(pos.x , pos.y);
	}
	}
	
}
void
LCD::rectangle_empty (uint16_t color,Rect format, Rect pos) //make the border of the rectangle 
{//to make the code better: " use a window HSA in the datasheet"
	Rect format_line;
	format_line.x=1;
	format_line.y=format.y;

	set_cursor(pos.x,pos.y);
	cs_.reset();
	write_index (0x0022);
	for(int j=0 ; j<format.x ; j++)
	{
		write_data(color);
	}
	cs_.set();
	rectangle_fill( color , format_line , pos);
	
	pos.x += format.x;
	rectangle_fill( color , format_line , pos);
	pos.y += format.y;
	pos.x -= format.x;

	set_cursor(pos.x,pos.y);
	
	cs_.reset();
	write_index (0x0022);
	for(int j=0 ; j<format.x ; j++)
	{
		write_data(color);
	}
	cs_.set();
	
}
void
LCD::blit_pixel ( uint16_t color , int x , int y)//blit a pixel if he is on the screen 
{
	if(x<0 || x>x_max ||y<0 || y>y_max){return;}	
	set_cursor(x,y);
	cs_.reset();
	write_index (0x0022);
	write_data(color);
	cs_.set();
}

void 
LCD::circle ( uint16_t color , int radius , Rect pos) //draw a circle (no problem if a part of the circle is not on the screen)
{
	int x=0;
	int y=radius;
	int m=5-4*radius;
	while(x<=y)
	{
		if(belong (x+pos.x , y+pos.y))		
		{blit_pixel(color , x+pos.x , y+pos.y);}
		
		if(belong ( y+pos.x , x+pos.y))
		{blit_pixel(color , y+pos.x , x+pos.y);}

		if(belong (-x+pos.x , y+pos.y))
		{blit_pixel(color , -x+pos.x , y+pos.y);}

		if(belong (-y+pos.x , x+pos.y))
		{blit_pixel(color , -y+pos.x , x+pos.y);}

		if(belong (x+pos.x , -y+pos.y))
		{blit_pixel(color , x+pos.x , -y+pos.y);}

		if(belong ( y+pos.x , -x+pos.y))
		{blit_pixel(color , y+pos.x , -x+pos.y);}

		if(belong (-x+pos.x , -y+pos.y))
		{blit_pixel(color , -x+pos.x , -y+pos.y);}

		if(belong (-y+pos.x , -x+pos.y))
		{blit_pixel(color , -y+pos.x , -x+pos.y);}
	
		if(m > 0)
		{
			y=y-1;
			m = m-8*y;
		}
		x=x+1;
		m=m+8*x+4;
	}
}
int 
LCD::belong (int x , int y) //test if the point A(x,y) belong to the screen 
{
	if(0<=x && x<=x_max && 0<=y && y<=y_max)
	{return 1;}
	return 0;
}
void 
LCD::draw_char( uint16_t color ,char caract, Rect pos)//draw a caracter
{
	if(pos.x+8> x_max || pos.y+16>y_max){return;}
	caract =caract-32;
	for( int i = 0 ; i<=16 ; i++)
	{
		for( int j =7 ; j>=0 ; j--)
		{
			if(Ascii[caract][i]&(1<<j)){blit_pixel(color,8-j+pos.x,i+pos.y);}
		}
	}
}
void
LCD::draw_sentence( uint16_t color ,const char *sentence, Rect pos) //write a sentence center on pos 
{
	char car_act='A'; //carat actuel
	int i=0;
	car_act =sentence[i];	
	while(car_act!='\0')
	{
		i++;
		car_act =sentence[i];
	}
	pos.x=pos.x-4*i;
	i=0;
	car_act =sentence[i];	
	while(car_act!='\0')
	{
		draw_char (color ,car_act,pos);
		pos.x=pos.x+8;
		i++;
		car_act =sentence[i];
	}
}
int
LCD::draw_sentence_left( uint16_t color ,const char *sentence, Rect pos) //write a sentence non center 
{
	char car_act='A'; //carat actuel
	int i=0;
	car_act =sentence[i];	
	while(car_act!='\0')
	{
		i++;
		car_act =sentence[i];
	}
	i=0;
	car_act =sentence[i];	
	while(car_act!='\0')
	{
		draw_char (color ,car_act,pos);
		pos.x=pos.x+8;
		i++;
		car_act =sentence[i];
	}
	return pos.x+8;
}























