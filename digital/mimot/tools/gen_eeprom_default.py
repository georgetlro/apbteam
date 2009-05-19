# mimot - Mini motor control, with motor driver. {{{
#
# Copyright (C) 2012 Nicolas Schodet
#
# APBTeam:
#        Web: http://apbteam.org/
#      Email: team AT apbteam DOT org
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
#
# }}}
from mimot import Proto
import mimot.init
import sys

if len (sys.argv) != 3 or sys.argv[1] not in ('host', 'target'):
    print >> sys.stderr, "usage: %s <host|target> <robot>" % sys.argv[0]
    sys.exit (1)

target, robot = sys.argv[1:]

init = dict (host = mimot.init.host,
        target = mimot.init.target)[target]
if robot not in init:
    print >> sys.stderr, "unknown robot"
    sys.exit (1)

proto = Proto (None, **init[robot])

params = proto.param

template = """\
/* Autogenerated file, do not edit! */

#ifdef EEPROM_DEFAULTS_KEY
#  error "duplicated definition"
#endif

#define EEPROM_DEFAULTS_KEY {key:#x}

struct eeprom_t PROGMEM eeprom_defaults =
{defaults};
"""

def param (name, offset = 0, m = None):
    if m:
        mname = m + '_' + name
        if mname in params:
            name = mname
    value = params[name]
    if offset:
        value = int (round (value * (1 << offset)))
    return '0x%x /* %s */' % (value, name)

def j (*params):
    return '{\n  ' + ',\n  '.join (p.replace ('\n', '\n  ')
            for p in params) + '\n}'

indexes = [ 'a%d' % i for i in xrange (2) ]

key = 0x03
defaults = j (
        'EEPROM_DEFAULTS_KEY',
        j (*[j (
            param ('speed_max', m = m),
            param ('speed_slow', m = m),
            param ('acc', offset = 8, m = m))
            for m in indexes]),
        j (*[j (
            param ('kp', offset = 8, m = m),
            param ('ki', offset = 8, m = m),
            param ('kd', offset = 8, m = m),
            param ('e_sat', m = m),
            param ('i_sat', m = m),
            param ('d_sat', m = m))
            for m in indexes]),
        j (*[j (
            param ('bd_error_limit', m = m),
            param ('bd_speed_limit', m = m),
            param ('bd_counter_limit', m = m))
            for m in indexes]),
        j (*[j (
            param ('reverse', m = m))
            for m in indexes]),
        '0')

print template.format (key = key, defaults = defaults)
