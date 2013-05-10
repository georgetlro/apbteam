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
"""APBirthday cherry cannon."""
from utils.observable import Observable
from simu.utils.trans_matrix import TransMatrix
from simu.utils.vector import vector
import random
from math import pi
from simu.robots.apbirthday.model import back

class Cannon (Observable):

    cannon_hit = (700, 100)

    def __init__ (self, table, robot_position,
            arm_cyl, clamp_cyl, contacts, pot):
        Observable.__init__ (self)
        self.table = table
        self.robot_position = robot_position
        self.arm_cyl = arm_cyl
        self.clamp_cyl = clamp_cyl
        self.contacts = contacts
        self.pot = pot
        self.plate = None
        self.cherries = [ ]
        self.robot_position.register (self.__robot_position_notified)
        self.arm_cyl.register (self.__arm_notified)
        self.clamp_cyl.register (self.__arm_notified)
        self.pot.register (self.__pot_notified)

    def __robot_position_notified (self):
        if self.robot_position.pos is None:
            return
        m = TransMatrix ()
        m.translate (self.robot_position.pos)
        m.rotate (self.robot_position.angle)
        x = -back
        y = (50 - 35, -50 - 35)
        for i, c in enumerate (self.contacts):
            s = True
            sensor_pos = m.apply ((x, y[i]))
            for o in self.table.obstacles:
                if (o.pos is not None and o.pos[1] > 0
                        and hasattr (o, 'inside') and o.inside (sensor_pos)):
                    s = False
                    break
            if s != self.contacts[i].state:
                self.contacts[i].state = s
                self.contacts[i].notify ()

    def __arm_notified (self):
        if self.robot_position.pos is None:
            return
        if (self.plate is None and self.arm_cyl.pos > 0.9
                and self.clamp_cyl.pos > 0.9):
            # Pick plate.
            self.plate = self.table.nearest (self.__plate_drop_point (),
                    level = 0, max = 50)
            if self.plate is not None:
                self.plate.pos = None
                self.plate.notify ()
        elif self.plate is not None and self.clamp_cyl.pos < 0.9:
            # Drop plate.
            self.plate.pos = self.__plate_drop_point ()
            self.plate.angle = self.robot_position.angle
            self.plate.notify ()
            self.plate = None
        elif (self.plate is not None and self.plate.cherries and
                self.arm_cyl.pos < .1):
            # Load cherries.
            self.cherries.extend (self.plate.cherries)
            self.plate.cherries = [ ]
            self.__pot_notified ()
        self.notify ()

    def __pot_notified (self):
        self.firing = self.pot.wiper[0] > .5
        if self.cherries and self.firing:
            m = TransMatrix ()
            m.translate (self.robot_position.pos)
            m.rotate (self.robot_position.angle)
            hit = vector (*m.apply (self.cannon_hit))
            for c in self.cherries:
                c.pos = hit + vector.polar (random.uniform (-pi, pi),
                        random.uniform (0, 50))
                c.notify ()
            self.table.cherries.cherries.extend (self.cherries)
            self.table.cherries.notify ()
            self.cherries = [ ]
        self.notify ()

    def __plate_drop_point (self):
        return (vector (self.robot_position.pos)
                + vector (-back - 85, -35).rotate (self.robot_position.angle))

