#ifndef simu_report_host_hh
#define simu_report_host_hh
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
#include "ucoolib/arch/host/host.hh"
#include "defs.hh"

/// Report internal information to be displayed by simulator interface.
class SimuReport
{
  public:
    /// Constructor.
    SimuReport (ucoo::Host &host);
    /// Destructor.
    ~SimuReport ();
    /// Report positions.
    void pos (vect_t *pos, int pos_nb, uint8_t id);
    /// Report path.
    void path (const vect_t *pos, int pos_nb);
    /// Debug draw.
    void draw_start ();
    void draw_send ();
    void draw_circle (const vect_t &p, int radius, uint8_t color);
    void draw_segment (const vect_t &p1, const vect_t &p2, uint8_t color);
    void draw_point (const vect_t &p, uint8_t color);
    void draw_number (const vect_t &p, int number);
  private:
    ucoo::mex::Node &node_;
    ucoo::mex::mtype_t pos_mtype_;
    ucoo::mex::mtype_t path_mtype_;
    ucoo::mex::mtype_t draw_mtype_;
    ucoo::mex::Msg *draw_msg_;
};

#endif // simu_report_host_hh
