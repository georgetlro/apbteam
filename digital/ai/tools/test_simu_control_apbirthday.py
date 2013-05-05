# io - Input & Output with Artificial Intelligence (ai) support on AVR. {{{
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
from test_simu import TestSimu, run
from Tkinter import *
import math

import io_hub.apbirthday

class TestSimuControl (TestSimu):
    """Interface with extra control."""

    def __init__ (self, robot_class, *args):
        TestSimu.__init__ (self, robot_class, *args,
                color_switch_set_pos = True)
        self.io = self.robots[0].io
        self.asserv = self.robots[0].asserv
        self.robot_model = self.robots[0].model
        self.io.output (io_hub.apbirthday.output_mask (
            'cake_arm_in', 'cake_push_far_in', 'cake_push_near_in',
            'cherry_plate_down', 'gift_in'), 'toggle')

    def create_widgets (self):
        TestSimu.create_widgets (self)
        self.control_frame = Frame (self)
        self.control_frame.pack (side = 'left', before = self.table_view,
                fill = 'y')
        Button (self.control_frame, text = 'FSM step', padx = 0, pady = 0,
                command = self.fsm_debug).pack ()
        Button (self.control_frame, text = 'Asserv block', padx = 0, pady = 0,
                command = self.asserv_block).pack ()
        def out_button (name, *toggle):
            def command ():
                self.io.output (io_hub.apbirthday.output_mask (*toggle),
                        'toggle')
            button = Button (self.control_frame, text = name,
                    padx = 0, pady = 0, command = command)
            button.pack ()
        out_button ('Arm in/out', 'cake_arm_in', 'cake_arm_out')
        out_button ('Push far in/out', 'cake_push_far_in', 'cake_push_far_out')
        out_button ('Push near in/out', 'cake_push_near_in', 'cake_push_near_out')
        out_button ('Plate arm up/down', 'cherry_plate_up', 'cherry_plate_down')
        out_button ('Plate clamp', 'cherry_plate_clamp_close', 'cherry_plate_clamp_open')
        out_button ('Gift', 'gift_in', 'gift_out')
        cannon_var = IntVar ()
        def cannon_cmd ():
            self.io.potentiometer (0, 256 if cannon_var.get () else 0)
        Checkbutton (self.control_frame, text = 'Fire!', indicatoron = 0,
                variable = cannon_var, command = cannon_cmd).pack ()
        self.backward_var = IntVar ()
        self.backward_button = Checkbutton (self.control_frame,
                text = 'Backward', variable = self.backward_var)
        self.backward_button.pack ()
        self.goto_var = IntVar ()
        self.goto_button = Checkbutton (self.control_frame,
                text = 'Goto FSM', variable = self.goto_var)
        self.goto_button.pack ()
        self.table_view.bind ('<1>', self.move)
        self.table_view.bind ('<3>', self.orient)

    def fsm_debug (self):
        self.io.fsm_debug ()

    def asserv_block (self):
        self.asserv.block ()

    def move (self, ev):
        pos = self.table_view.screen_coord ((ev.x, ev.y))
        if self.goto_var.get ():
            self.io.goto (pos[0], pos[1], self.backward_var.get ())
        else:
            self.asserv.goto (pos[0], pos[1], self.backward_var.get ())

    def orient (self, ev):
        x, y = self.table_view.screen_coord ((ev.x, ev.y))
        robot_pos = self.robot_model.position.pos
        if robot_pos is not None:
            a = math.atan2 (y - robot_pos[1], x - robot_pos[0])
            self.asserv.goto_angle (a)

if __name__ == '__main__':
    run ('apbirthday', TestSimuControl)
