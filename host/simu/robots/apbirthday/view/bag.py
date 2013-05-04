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
"""APBirthday bag of views."""
from simu.view.switch import Switch
from simu.view.distance_sensor import DistanceSensor
from simu.view.distance_sensor_us import DistanceSensorUS
from simu.view.path import Path
from simu.view.pos_report import PosReport
from simu.robots.apbirthday.view.robot import Robot

class Bag:

    def __init__ (self, table, actuator_view, sensor_frame, model_bag):
        self.jack = Switch (sensor_frame, model_bag.jack, 'Jack')
        self.color_switch = Switch (sensor_frame, model_bag.color_switch,
                'Color')
        self.strat_switch = Switch (sensor_frame, model_bag.strat_switch,
                'Strat')
        self.robot_nb_switch = Switch (sensor_frame,
                model_bag.robot_nb_switch, 'Nb robots')
        self.robot = Robot (table, model_bag.position, model_bag.cake_arm,
                model_bag.cannon, model_bag.gifts_arm, model_bag.ballon)
        self.distance_sensor = [DistanceSensorUS (self.robot, ds)
                for ds in model_bag.distance_sensor]
        self.cake_front = DistanceSensor (self.robot, model_bag.cake_front)
        self.cake_back = DistanceSensor (self.robot, model_bag.cake_back)
        self.path = Path (table, model_bag.path)
        self.pos_report = PosReport (table, model_bag.pos_report)

