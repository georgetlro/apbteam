# io-hub - Modular Input/Output. {{{
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
"""Tools to interface with apbirthday io-hub."""
from Tkinter import *
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy

import io_hub
from utils.init_proto import init_proto

import io_hub.apbirthday

class InterAPBirthday (Frame):

    def __init__ (self, master = None):
        Frame.__init__ (self, master)
        self.pack (expand = True, fill = 'both')
        self.create_widgets ()
        self.update ()
        self.io = init_proto ('apbirthday', io_hub.Proto)
        self.io.async = True
        self.io.register_stats (self.handle_stats)
        def io_read (*args):
            self.io.proto.read ()
            self.io.proto.sync ()
        def io_timeout ():
            self.io.proto.sync ()
            self.after (100, io_timeout)
        self.tk.createfilehandler (self.io, READABLE, io_read)
        io_timeout ()

    def create_widgets (self):
        Button (self, text = 'Reset', command = self.reset).pack ()
        # Outputs.
        output_frame = Frame (self)
        output_frame.pack (side = LEFT, fill = Y)
        row = 0
        def make_setreset (index):
            def setreset ():
                val = self.output_var[index].get ()
                self.io.output (1 << index, val)
            return setreset
        def make_toggle (*index):
            def toggle ():
                transient = transient_scale.get ()
                mask = 0
                for i in index:
                    if not transient:
                        var = self.output_var[i]
                        val = 1 - var.get ()
                        var.set (val)
                    mask |= 1 << i
                if transient:
                    self.io.output_transient (mask, transient)
                else:
                    self.io.output (mask, 'toggle')
            return toggle
        def common (a, b):
            if a is None or b is None: return None
            a1, a2 = a.rsplit ('_', 1)
            b1, b2 = b.rsplit ('_', 1)
            if a1 != b1: return None
            def cmp (a, b):
                return (a == 'up' and b == 'down'
                        or a == 'in' and b == 'out'
                        or a == 'open' and b == 'close')
            if cmp (a2, b2) or cmp (b2, a2):
                return a1
            return None
        previous = None
        self.output_var = [ ]
        for i, name in enumerate (io_hub.apbirthday.outputs):
            var = IntVar ()
            self.output_var.append (var)
            button = Checkbutton (output_frame, indicatoron = 0,
                    text = '%d: %s' % (i, name), command = make_setreset (i),
                    variable = var)
            button.grid (column = 0, row = row, sticky = 'nsew')
            c = common (name, previous)
            if c:
                button = Button (output_frame, text = c, command =
                        make_toggle (i - 1, i))
                button.grid (column = 1, row = row - 1, rowspan = 2,
                        sticky = 'nsew')
            previous = name
            row += 1
        transient_scale = Scale (output_frame, from_ = 0, to = 250,
                label = 'Transient', orient = HORIZONTAL)
        transient_scale.grid (column = 1, row = row)
        row += 1
        # Misc.
        misc_frame = Frame (self)
        misc_frame.pack (side = LEFT, fill = Y)
        #  Pressure.
        frame = LabelFrame (misc_frame, text = 'Pressure')
        frame.pack ()
        def pressure_update (val):
            pressure_scale.configure (label = hex (int (val)))
        pressure_max = 4096
        pressure_scale = Scale (frame, from_ = 0, to = pressure_max -
                pressure_max / 16, resolution = pressure_max / 32,
                orient = HORIZONTAL, showvalue = 0, command = pressure_update)
        pressure_scale.pack ()
        pressure_scale.set (0x0c00)
        def pressure_set ():
            val = pressure_scale.get ()
            self.io.pressure (val)
        button = Button (frame, text = 'Set', command = pressure_set,
                padx = 0, pady = 0)
        button.pack ()
        #  Cannon.
        frame = LabelFrame (misc_frame, text = 'Cannon')
        frame.pack ()
        cannon_speed_scale = Scale (frame, from_ = 256, to = 0,
                orient = HORIZONTAL)
        cannon_speed_scale.pack ()
        frame = Frame (frame)
        frame.pack ()
        def cannon_speed_set ():
            self.io.potentiometer (1, cannon_speed_scale.get ())
        button = Button (frame, text = 'Set', command = cannon_speed_set,
                padx = 0, pady = 0)
        button.pack (side = LEFT)
        def cannon_fire ():
            self.io.potentiometer (0, (0, 256)[self.cannon_fire_var.get ()])
        self.cannon_fire_var = IntVar ()
        button = Checkbutton (frame, indicatoron = 0, text = 'Fire!',
                command = cannon_fire, variable = self.cannon_fire_var)
        button.pack (side = LEFT)
        # Graph.
        graph_frame = Frame (self)
        graph_frame.pack (side = LEFT, fill = BOTH, expand = 1)
        self.graph_figure = Figure ()
        self.graph_axes = self.graph_figure.add_subplot (111)
        self.graph_canvas = FigureCanvasTkAgg (self.graph_figure,
                master = graph_frame)
        self.graph_canvas.show ()
        self.graph_canvas.get_tk_widget ().pack (fill = BOTH, expand = 1)
        #  Graph information.
        class Graph:
            def __init__ (self, stat):
                self.var = IntVar ()
                self.line = None
                self.data = None
                self.stat = stat
        class GraphStat:
            def __init__ (self, stat):
                self.stat = stat
                self.usage = 0
        self.graphs = { }
        self.graph_enabled = False
        #  Buttons.
        self.graphs['pressure'] = Graph (GraphStat ('pressure'))
        button = Checkbutton (graph_frame, text = 'Pressure',
                variable = self.graphs['pressure'].var,
                command = self.graph_update)
        button.pack ()
        gstat = GraphStat ('us')
        for i in xrange (4):
            stat = 'us%d' % i
            self.graphs[stat] = Graph (gstat)
            button = Checkbutton (graph_frame, text = 'US %d' % i,
                    variable = self.graphs[stat].var,
                    command = self.graph_update)
            button.pack ()
        gstat = GraphStat ('cake')
        for i in xrange (2):
            stat = 'cake%d' % i
            self.graphs[stat] = Graph (gstat)
            button = Checkbutton (graph_frame, text = 'Cake %d' % i,
                    variable = self.graphs[stat].var,
                    command = self.graph_update)
            button.pack ()

    def reset (self):
        mask = 0
        for i in xrange (len (io_hub.apbirthday.outputs)):
            self.output_var[i].set (0)
            mask |= 1 << i
        self.io.output (mask, 'clear')
        self.io.pressure (0)
        self.cannon_fire_var.set (0)
        self.io.potentiometer (0, 0)

    def graph_update (self):
        graph_sample_rate = 1
        graph_history = 250 * 4
        for k, g in self.graphs.iteritems ():
            val = g.var.get ()
            if g.line is None and val:
                g.stat.usage += 1
                if g.stat.usage == 1:
                    self.io.stats (g.stat.stat, graph_sample_rate)
                g.data = numpy.zeros (graph_history)
                g.line, = self.graph_axes.plot (g.data, label = k)
            elif g.line is not None and not val:
                g.stat.usage -= 1
                if g.stat.usage == 0:
                    self.io.stats (g.stat.stat, 0)
                self.graph_axes.lines.remove (g.line)
                g.data, g.line = None, None
        if not self.graph_enabled and self.graph_axes.lines:
            self.graph_draw ()

    def graph_draw (self):
        ymin, ymax = 0, 0
        for g in self.graphs.itervalues ():
            if g.line is not None:
                g.line.recache ()
                ymin = min (ymin, g.data.min ())
                ymax = max (ymax, g.data.max ())
        self.graph_axes.set_ylim (ymin * 1.1, ymax * 1.1)
        self.graph_canvas.draw ()
        if self.graph_axes.lines:
            self.graph_enabled = True
            self.after (200, self.graph_draw)
        else:
            self.graph_enabled = False

    def handle_stats (self, stats, *values):
        if len (values) != 1:
            stats = [ '%s%d' % (stats, i) for i in xrange (len (values)) ]
        else:
            stats = (stats, )
        for stat, value in zip (stats, values):
            g = self.graphs[stat]
            if g.line is not None:
                g.data[0:-1] = g.data[1:]
                g.data[-1] = value

if __name__ == '__main__':
    app = InterAPBirthday ()
    app.mainloop ()
