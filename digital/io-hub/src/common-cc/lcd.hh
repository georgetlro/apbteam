#ifndef lcd_hh
#define lcd_hh
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
#include "i2c_queue.hh"

#include "defs.hh"

/// Interface to LCD board.
class LCD : public I2cQueue::Slave
{
  public:
    /// Constructor.
    LCD (I2cQueue &queue);
    /// See I2cQueue::Slave::recv_status.
    void recv_status (const uint8_t *status);
    /// Send team color.
    void team_color (uint8_t r, uint8_t g, uint8_t b);
    /// Send chrono if changed.
    void chrono (int s);
    /// Send message.
    void message (const char *msg);
    /// Send robot position.
    void robot_position (const Position &pos);
    /// Send obstacles.
    void obstacles (const vect_t *pos, int pos_nb);
    /// Send target position.
    void target (const vect_t &pos);
  private:
    int last_chrono_sent_;
};

#endif // lcd_hh
