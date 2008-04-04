# inter - Robot simulation interface. {{{
#
# Copyright (C) 2008 Nicolas Schodet
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
"""Inter, coupled with a mex Node."""
if __name__ == '__main__':
    import sys
    sys.path.append (sys.path[0] + '/../mex')

from inter import Inter
from Tkinter import *
from mex.node import Node

class InterNode (Inter):
    """Inter, coupled with a mex Node."""

    # There is 900 tick per seconds, to permit AVR to have a 4.44444 ms
    # period.
    TICK = 900.0

    def __init__ (self):
	Inter.__init__ (self)
	self.node = Node ()
	self.tk.createfilehandler (self.node, READABLE, self.read)
	self.step_after = None

    def createWidgets (self):
	Inter.createWidgets (self)
	self.nowLabel = Label (self.rightFrame, text = 'Now: 0 s')
	self.nowLabel.pack ()
	self.stepButton = Button (self.rightFrame, text = 'Step',
		command = self.step)
	self.stepButton.pack ()
	self.stepSizeScale = Scale (self.rightFrame, orient = HORIZONTAL,
		from_ = 0.1, to = 1.0, resolution = 0.1)
	self.stepSizeScale.pack ()
	self.playVar = IntVar ()
	self.playButton = Checkbutton (self.rightFrame, variable =
		self.playVar, text = 'Play', command = self.play)
	self.playButton.pack ()

    def step (self):
	"""Do a step.  Signal to the Hub we are ready to wait to the next step
	date."""
	self.node.wait_async (self.node.date
		+ int (self.stepSizeScale.get () * self.TICK))
	self.step_after = None

    def play (self):
	"""Activate auto-steping."""
	if self.playVar.get ():
	    if self.step_after is None:
		self.step ()
	    self.stepButton.configure (state = DISABLED)
	else:
	    if self.step_after is not None:
		self.after_cancel (self.step_after)
		self.step_after = None
	    self.stepButton.configure (state = NORMAL)

    def read (self, file, mask):
	"""Handle event on the Node."""
	self.node.read ()
	if self.node.sync ():
	    self.synced = True
	    self.nowLabel.configure (text = 'Now: %.1f s' % (self.node.date /
		self.TICK))
	    self.update ()
	    if self.playVar.get ():
		self.step_after = self.after (int (self.stepSizeScale.get ()
		    * self.TICK), self.step)

if __name__ == '__main__':
    import mex.hub
    import mex.forked
    h = mex.hub.Hub (min_clients = 1)
    fh = mex.forked.Forked (h.wait)
    try:
	app = InterNode ()
	app.mainloop()
    finally:
	fh.kill ()
	import time
	time.sleep (1)
