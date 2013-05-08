# simu - Robot simulation. {{{
#
# Copyright (C) 2013 Nicolas Schodet
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
"""APBirthday gifts arm model."""
from utils.observable import Observable
from simu.utils.vector import vector
from math import pi
from simu.robots.apbirthday.model import side

class GiftsArm (Observable):

    def __init__ (self, table, robot_position, arm_cyl):
        Observable.__init__ (self)
        self.table = table
        self.robot_position = robot_position
        self.arm_cyl = arm_cyl
        self.arm_cyl.register (self.__arm_notified)

    def __arm_notified (self):
        if self.arm_cyl.pos > .9:
            push_point = (vector (self.robot_position.pos)
                    + vector.polar (self.robot_position.angle - pi / 2, side + 100)
                    - vector.polar (self.robot_position.angle, 100))
            gift = self.table.nearest (push_point, level = 0, max = 150)
            if gift is not None and hasattr (gift, 'state'):
                gift.state = True
                gift.notify ()
        self.notify ()
