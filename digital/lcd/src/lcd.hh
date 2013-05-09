#ifndef lcd_hh
#define lcd_hh
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
#include "ucoolib/common.hh"
#include "ucoolib/hal/gpio/gpio.hh"

/// Control LCD.

struct Rect
{
	int x;
	int y;
};

class LCD
{
  public:
    static const int x_max = 320, y_max = 240;
  public:
    /// Constructor.
    LCD ();
    /// Turn display on.
    void on ();
    /// Turn display off.
    void off ();
    /// Clear the whole screen with the same color (default to black).
    void clear (uint16_t color = 0);
    /// Fill screen with colors.
    void rainbow ();
    /// Convert from RGB to hardware color format.
    static inline uint16_t color (uint8_t r, uint8_t g, uint8_t b)
    {
        return (r >> 3 << 11) | (g >> 2 << 5) | (b >> 3);
    }
    ///draw a pixel if he belong to the screen
    void blit_pixel ( uint16_t color , int x , int y);
    ///make a border of a rectangle 
    void rectangle_empty (uint16_t color,Rect format, Rect pos);
    ///make a filled rectangle with one color 
    void rectangle_fill (uint16_t color,Rect format, Rect pos);
    ///draw a circle 
    void circle ( uint16_t color , int radius , Rect pos); //position of the center of the circle 
    ///write a caracter 
    void draw_char( uint16_t color ,char caract, Rect pos); 
    ///write a sentence center on pos 
    void draw_sentence( uint16_t color ,const char *sentence, Rect pos);
  private:
    void set_cursor (int x, int y);
    void write_index (uint16_t index);
    void write_data (uint16_t data);
    uint16_t read_data ();
    void write_reg (uint16_t index, uint16_t data);
    uint16_t read_reg (uint16_t index);
    int belong ( int x, int y); //does the point belong to the screen?
  private:
    ucoo::Gpio cs_, rs_, wr_, rd_, reset_, bl_;
};
#endif // lcd_hh
