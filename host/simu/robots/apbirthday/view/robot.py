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
"""APBirthday robot view."""
import simu.inter.drawable
from simu.view.table_eurobot2013 import PINK, colors
import math

COLOR_ROBOT = '#000000'
COLOR_AXES = '#202040'
COLOR_CANNON = '#808080'
COLOR_BALLON = '#ff0000'

class Robot (simu.inter.drawable.Drawable):

    def __init__ (self, onto, position_model, cake_arm_model, cannon_model,
            gifts_arm_model, ballon_model):
        """Construct and make connections."""
        simu.inter.drawable.Drawable.__init__ (self, onto)
        self.position_model = position_model
        self.position_model.register (self.__position_notified)
        self.cake_arm_model = cake_arm_model
        self.cake_arm_model.register (self.update)
        self.cannon_model = cannon_model
        self.cannon_model.register (self.update)
        self.gifts_arm_model = gifts_arm_model
        self.gifts_arm_model.register (self.update)
        self.ballon_model = ballon_model
        self.ballon_model.register (self.update)

    def __position_notified (self):
        """Called on position modifications."""
        self.pos = self.position_model.pos
        self.angle = self.position_model.angle
        self.update ()

    def draw (self):
        """Draw the robot."""
        self.reset ()
        if self.pos is not None:
            self.trans_translate (self.pos)
            self.trans_rotate (self.angle)
            # Draw plate.
            plate = self.cannon_model.plate
            f = self.cannon_model.arm_cyl.pos
            if plate is not None:
                self.draw_rectangle ((-108 - f * 170, 85), (-108, -85),
                        fill = PINK)
                self.draw_rectangle ((-108 - f * 148, 85 - 22),
                        (-108 - f * 22, -85 + 22), fill = PINK)
                for c in plate.cherries:
                    if c.pos:
                        self.draw_circle ((-108 - f * (c.pos[0] + 85),
                            c.pos[1]), c.radius, fill = colors[c.color])
                self.draw_rectangle ((-108 - f * 170, 85),
                        (-108 - f * 170 - (1 - f) * 22, -85), fill = PINK)
            # Draw robot body.
            self.draw_polygon ((102, 140), (102, -140), (-108, -140),
                    (-108, 70), (-58, 140), fill = COLOR_ROBOT)
            self.draw_circle ((70, self.cannon_model.cannon_hit[1]), 20,
                    fill = COLOR_CANNON)
            # Draw Robot axis.
            self.draw_line ((-50, 0), (50, 0), fill = COLOR_AXES,
                    arrow = 'last')
            # Draw Robot wheels.
            f = 190 # Wheel spacing
            wr = 65 / 2 # Wheel radius
            self.draw_line ((0, +f / 2), (0, -f / 2), fill = COLOR_AXES)
            self.draw_line ((-wr, f / 2), (+wr, f / 2), fill = COLOR_AXES)
            self.draw_line ((-wr, -f / 2), (+wr, -f / 2), fill = COLOR_AXES)
            # Draw arm.
            m = self.cake_arm_model
            f = m.arm_cyl.pos
            for x, y, pos in ((m.far_x, m.far_y, m.far_cyl.pos),
                    (m.near_x, m.near_y, m.near_cyl.pos)):
                e = (y - 140) * f + 140
                gray = pos * 0x40
                fill = '#%02x%02x%02x' % (gray, gray, gray)
                self.draw_polygon ((x - 20, 140), (x - 20, e), (x + 20, e),
                        (x + 20, 140), fill = fill)
            # Draw gifts arm.
            m = self.gifts_arm_model
            a = math.pi + m.arm_cyl.pos * math.pi / 6
            self.draw_line ((0, -140), (0 + 108 * math.cos (a),
                -140 + 108 * math.sin (a)))
            # Draw ballon.
            if self.ballon_model.pos > .1:
                self.draw_circle ((50, -100), self.ballon_model.pos * 75,
                        fill = COLOR_BALLON)
            # Extends.
            simu.inter.drawable.Drawable.draw (self)

