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
#include "lcd.hh"

#include "ucoolib/utils/bytes.hh"

#include <cstring>

LCD::LCD (I2cQueue &queue)
    : I2cQueue::Slave (queue, 0x20, 1), last_chrono_sent_ (-1)
{
}

void
LCD::recv_status (const uint8_t *status)
{
}

void
LCD::team_color (uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t buf[] = { 'c', r, g, b };
    send (buf, sizeof (buf));
}

void
LCD::chrono (int s)
{
    if (s != last_chrono_sent_)
    {
        last_chrono_sent_ = s;
        uint8_t t = s > 0 ? s : 0;
        uint8_t buf[] = { 't', t };
        send (buf, sizeof (buf));
    }
}

void
LCD::message (const char *msg)
{
    uint8_t buf[16];
    buf[0] = 'm';
    unsigned msg_len = std::strlen (msg) + 1;
    if (msg_len > sizeof (buf) - 1)
        msg_len = sizeof (buf) - 1;
    std::memcpy (buf + 1, msg, msg_len);
    buf[sizeof (buf) - 1] = 0;
    send (buf, 1 + msg_len);
}

void
LCD::robot_position (const Position &pos)
{
    uint8_t buf[] = { 'p',
        ucoo::bytes_unpack (pos.v.x, 1),
        ucoo::bytes_unpack (pos.v.x, 0),
        ucoo::bytes_unpack (pos.v.y, 1),
        ucoo::bytes_unpack (pos.v.y, 0),
    };
    send (buf, sizeof (buf), I2cQueue::TRANSIENT);
}

void
LCD::obstacles (const vect_t *pos, int pos_nb)
{
    uint8_t buf[2 + 4 * 4];
    int i = 0;
    buf[i++] = 'o';
    buf[i++] = pos_nb;
    for (int o = 0; o < pos_nb; o++)
    {
        buf[i++] = ucoo::bytes_unpack (pos[o].x, 1);
        buf[i++] = ucoo::bytes_unpack (pos[o].x, 0);
        buf[i++] = ucoo::bytes_unpack (pos[o].y, 1);
        buf[i++] = ucoo::bytes_unpack (pos[o].y, 0);
    }
    send (buf, i, I2cQueue::TRANSIENT);
}

void
LCD::target (const vect_t &pos)
{
    uint8_t buf[] = { 'n',
        ucoo::bytes_unpack (pos.x, 1),
        ucoo::bytes_unpack (pos.x, 0),
        ucoo::bytes_unpack (pos.y, 1),
        ucoo::bytes_unpack (pos.y, 0),
    };
    send (buf, sizeof (buf));
}

