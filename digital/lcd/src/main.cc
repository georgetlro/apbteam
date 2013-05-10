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

#include "ucoolib/arch/arch.hh"
#include "ucoolib/utils/delay.hh"
#include <libopencm3/stm32/f4/rcc.h>
#include <cstring>


#include "ucoolib/hal/i2c/i2c.hh"
#include "ucoolib/utils/crc.hh"
#include "ucoolib/utils/bytes.hh"
//uint16_t ucoo::bytes_pack (arg 1 ,arg 2 );



static char i2c_color[3];
static int i2c_time;
static char i2c_cmd[100]="...";
static Rect pos_r;//position du robot 
static int nb_obs=0;//nb d'obstacle 
static Rect pos_obs[4];
static Rect pos_r_n; //next position of the robot 

static const int i2c_status_size = 3;
static const int i2c_command_size = 16;

static uint8_t i2c_seq;

static bool i2c_received;

/// Handle a command received by I2C.
void
i2c_handle (LCD &lcd, const char *buf, int size)
{
    // Command too small.
    if (size < 3)
        return;
    // Check CRC.
    if (ucoo::crc8_compute ((const uint8_t *) buf + 1, size - 1) != buf[0])
        return;
    // Handle sequence number.
    if (buf[1] != 0 && buf[1] == i2c_seq)
        // Duplicated command.
        return;
    // OK, now handle command.
    char cmd = buf[2];
    const char *arg = &buf[3];
    int arg_nb = size - 3;
    int j;//for incrementation
    int x;
    int y;
    switch (cmd)
    {
    case 'c':
        // Team color.
        if (arg_nb != 3)
            return;
		i2c_color[0]=arg[0];
		i2c_color[1]=arg[1];
		i2c_color[2]=arg[2];
        break;
    case 't':
		if(arg[0]<0 || arg[0]>90){strcpy(i2c_cmd,"ERROR I2C time");}
		i2c_time=arg[0];
	break;
    case 'p': //position of the robot 
		x=ucoo::bytes_pack (arg[0] ,arg [1] )*0.10666;
		y=ucoo::bytes_pack (arg[2] ,arg [3] )*0.10666;
		if(lcd.belong (x,y))
		{pos_r.x=ucoo::bytes_pack (arg[0] ,arg [1] )*0.10666;
		pos_r.y=ucoo::bytes_pack (arg[2] ,arg [3] )*0.10666;}
		else{strcpy(i2c_cmd,"ERROR I2C position");}	
	break;
    case 'm': //message 
		strcpy(i2c_cmd,arg);
	break;	
		
    case 'o': //obstacle (barrier)
		j=0;		
		nb_obs=arg[0];
		for(int i=1 ; i<=nb_obs*4 ; i=i+4)
		{
			x=ucoo::bytes_pack (arg[i] ,arg [i+1] )*0.10666;
			y=ucoo::bytes_pack (arg[i+2] ,arg [i+3] )*0.10666;
			if(lcd.belong (x,y))
			{ pos_obs[j].x=ucoo::bytes_pack (arg[i] ,arg [i+1] )*0.10666;

			pos_obs[j].y=ucoo::bytes_pack (arg[i+2] ,arg [i+3] )*0.10666;
			j++;}
			else{strcpy(i2c_cmd,"ERROR I2C obstacle");}
		}
	break;
    case 'n'://next position of the robot 
	x=ucoo::bytes_pack (arg[0] ,arg [1] )*0.10666;
	y=ucoo::bytes_pack (arg[2] ,arg [3] )*0.10666;
	if(lcd.belong (x,y))
	{pos_r_n.x=ucoo::bytes_pack (arg[0] ,arg [1] )*0.10666;
	pos_r_n.y=ucoo::bytes_pack (arg[2] ,arg [3] )*0.10666;}
	break;
    default:
        // Unknown command.
        return;
    }
    i2c_received = true;
    // Acknowledge.
    if (buf[1] != 0)
        i2c_seq = buf[1];
}

/// Poll I2C interface for commands and update status.
void
i2c_poll (LCD &lcd, ucoo::I2cSlaveDataBuffer &i2c_data)
{
    char buf[i2c_command_size];
    int size;
    // Handle incoming commands.
    while ((size = i2c_data.poll (buf, sizeof (buf))))
        i2c_handle (lcd, buf, size);
    // Update status.
    char status[i2c_status_size];
    status[1] = i2c_seq;
    status[2] = 0;
    status[0] = ucoo::crc8_compute ((const uint8_t *) &status[1],
                                    sizeof (status) - 1);
    i2c_data.update (status, sizeof (status));
}
void 
draw_bar (LCD lcd)//draw barrier (make a red circle where is approximatly the barrier)
{
	for(int i=0; i<nb_obs ;i++)
	{
		lcd.circle (LCD::color(255,0,0),10 ,pos_obs[i]);
	}
}

void
draw_robot (LCD lcd) //draw of the robot and his barriers
{
	Rect pos_rob;pos_rob.x=pos_r.x-12;pos_rob.y=pos_r.y-12;//center the bild of the robot 
	Rect size;size.x=24;size.y=24; //size of the robot
	Rect pos_rob_n;pos_rob_n.x=pos_r_n.x-2;pos_rob_n.y=pos_r_n.y-2;//center the bild of the next position of the robot 
	Rect size_dest;size_dest.x=5;size_dest.y=5;//size of the robot next position 
	lcd.rectangle_fill (LCD::color ( 100,100,100),size, pos_rob);
	draw_bar(lcd);
	lcd.rectangle_fill (LCD::color (0,255,0),size_dest ,pos_rob_n);
	
}
void 
draw_time (LCD lcd) //write the remaining time 
{	

		
	Rect pos_u;pos_u.x=300;pos_u.y=223;//digit two  
	Rect pos_d;pos_d.x=292;pos_d.y=223;//digit one 
	char dizaine = i2c_time/10 +48;
	char unite = i2c_time%10 +48;
	if(i2c_time<0){lcd.draw_sentence (0,"ERROR",pos_d);}
	else{
	if(i2c_time/10!=0){lcd.draw_char( 0 ,dizaine,pos_d);}
	lcd.draw_char( 0 ,unite,pos_u);}

}
void 
draw_table (LCD lcd) //draw the table on the screen 
{
	Rect pos_i2c_cmd;pos_i2c_cmd.x=160;pos_i2c_cmd.y=223;
	char commande [100]="msg: ";
	strcat(commande, i2c_cmd);
	Rect pos_cake;pos_cake.x=160;pos_cake.y=0; //position of the center of the cake 
		
	Rect pos_table; pos_table.x=43;pos_table.y=0; //position of the table on the screen
	Rect for_table; for_table.x= 234; for_table.y=214;//size of the table 
		
	Rect pos_cons; pos_cons.x=0;pos_cons.y=214;//position of the console 
	Rect for_cons; for_cons.x= 320; for_cons.y=27;//size of the console 
	
	Rect for_base;for_base.x=43;for_base.y=43;//size of a base 

	Rect pos_base_b_a;pos_base_b_a.x=0;pos_base_b_a.y=0; //position of the 3 blue bases
	Rect pos_base_b_b;pos_base_b_b.x=0;pos_base_b_b.y=87;
	Rect pos_base_b_c;pos_base_b_c.x=0;pos_base_b_c.y=172;
	
	Rect pos_base_r_a;pos_base_r_a.x=277;pos_base_r_a.y=0;//position of the 3 red bases
	Rect pos_base_r_b;pos_base_r_b.x=277;pos_base_r_b.y=87;
	Rect pos_base_r_c;pos_base_r_c.x=277;pos_base_r_c.y=172;

	Rect pos_base_n_a;pos_base_n_a.x=0;pos_base_n_a.y=44; //position of the 4 white bases
	Rect pos_base_n_b;pos_base_n_b.x=0;pos_base_n_b.y=130;
	Rect pos_base_n_c;pos_base_n_c.x=277;pos_base_n_c.y=44;
	Rect pos_base_n_d;pos_base_n_d.x=277;pos_base_n_d.y=130;
	
	Rect pos_color;pos_color.x=10;pos_color.y=223;//position of the team's color rectangle
	Rect for_color;for_color.x=10;for_color.y=10;//size of the rectangle 

	lcd.rectangle_fill (LCD::color(255,255,0),for_table,pos_table);
	
	lcd.circle (LCD::color(255,200,255), 53 , pos_cake);//pink circle of the cake
	lcd.rectangle_fill (0xffff , for_cons , pos_cons);//the white rectangle
	ucoo::delay_ms (15);
	lcd.rectangle_fill (LCD::color(0,0,255) , for_base, pos_base_b_a);
	lcd.rectangle_fill (LCD::color(0,0,255) , for_base, pos_base_b_b);
	lcd.rectangle_fill (LCD::color(0,0,255) , for_base, pos_base_b_c);

	lcd.rectangle_fill (LCD::color(255,0,0) , for_base, pos_base_r_a);
	lcd.rectangle_fill (LCD::color(255,0,0) , for_base, pos_base_r_b);
	lcd.rectangle_fill (LCD::color(255,0,0) , for_base, pos_base_r_c);
	
	lcd.rectangle_fill (LCD::color(255,255,255) , for_base, pos_base_n_a);
	lcd.rectangle_fill (LCD::color(255,255,255) , for_base, pos_base_n_b);
	lcd.rectangle_fill (LCD::color(255,255,255) , for_base, pos_base_n_c);
	lcd.rectangle_fill (LCD::color(255,255,255) , for_base, pos_base_n_d);

	lcd.rectangle_fill (LCD::color(i2c_color[0],i2c_color[1],i2c_color[2]),for_color,pos_color);
	draw_time (lcd);
	lcd.draw_sentence (0,commande,pos_i2c_cmd);
}

int
main (int argc, const char **argv)
{
    ucoo::arch_init (argc, argv);
    rcc_peripheral_enable_clock (&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN
                                 | RCC_AHB1ENR_IOPBEN | RCC_AHB1ENR_IOPCEN);
    // I2C: B8: SCL, B9: SDA
    gpio_mode_setup (GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
    gpio_set_output_options (GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO8 | GPIO9);
    gpio_set_af (GPIOB, GPIO_AF4, GPIO8 | GPIO9);
    ucoo::I2cHard i2c (0);
    i2c.enable ();
    ucoo::I2cSlaveDataBufferSize<i2c_status_size, i2c_command_size> i2c_data;
    i2c.register_data (0x20, i2c_data);
    //Init global variable 
	i2c_color[0]=0;i2c_color[1]=0;i2c_color[2]=0;
	i2c_time=90;
	pos_r_n.x=160;pos_r_n.y=120;
	pos_r.x=160;pos_r.y=120;
    // Init.
	
    LCD lcd;
	//ucoo::delay_ms (1000);
	draw_table (lcd);
    // Wait orders.
    while (1)
    {	
        if (i2c_received)
        {
            draw_table (lcd);//draw the table
            draw_robot (lcd);//draw the robot and his destination on the table
            i2c_received = false;
        }

        i2c_poll (lcd, i2c_data);
        ucoo::delay_ms (4);
	
    }
}
