#ifndef fsm_hh
#define fsm_hh
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

extern "C" {
#define ANGFSM_NAME AI
#include "angfsm.h"
}

#ifdef TARGET_host
typedef unsigned State;
typedef unsigned Branch;
typedef uint16_t Event;
#else
typedef angfsm_AI_state_t State;
typedef angfsm_AI_branch_t Branch;
typedef angfsm_AI_event_t Event;
#endif

#endif // fsm_hh
