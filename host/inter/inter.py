from Tkinter import *
from drawable import *

from math import pi

class Robot (Drawable):
    """The robot."""

    def draw (self):
	self.reset ()
	self.trans_rotate (self.angle)
	self.trans_translate (self.pos)
	self.draw_polygon ((115, 30), (170, 85), (150, 127), (130, 145),
		(-25, 200), (-70, 200), (-70, -200), (-25, -200),
		(130, -145), (150, -127), (170, -85), (115, -30))
	axes_fill = '#404040'
	self.draw_line ((-70, 0), (170, 0), fill = axes_fill, arrow = LAST)
	f = 142 + 2 * 31.5 - 13.5
	wr = 63 / 2
	self.draw_line ((0, +f / 2), (0, -f / 2), fill = axes_fill)
	self.draw_line ((-wr, f / 2), (+wr, f / 2), fill = axes_fill)
	self.draw_line ((-wr, -f / 2), (+wr, -f / 2), fill = axes_fill)

class Table (Drawable):
    """The table and its elements."""

    def draw (self):
	# Redraw.
	self.reset ()
	# Table.
	self.draw_rectangle ((-22, -22 - 80), (3000 / 2, 2100 + 22), fill = '#ff1f1f')
	self.draw_rectangle ((3000 / 2, -22 - 80), (3000 + 22, 2100 + 22), fill = '#201fff')
	self.draw_rectangle ((0, 0), (3000, 2100), fill = '#a49d8b')
	self.draw_rectangle ((0, -22 - 80), (3000, -22), fill = '#a49d8b')
	self.draw_rectangle ((0, 2100 - 500), (500, 2100), fill = '#201fff')
	self.draw_rectangle ((3000 - 500, 2100 - 500), (3000, 2100), fill = '#ff1f1f')
	self.draw_line ((3000 / 2, -22 - 80), (3000 / 2, 2100 + 22))
	# Axes.
	self.draw_line ((0, 200), (0, 0), (200, 0), arrow = BOTH)
	# Beacons and baskets.
	self.draw_rectangle ((-22, 2100), (-22 - 80, 2100 + 80), fill = '#5b5b5d')
	self.draw_rectangle ((-22, 1050 - 40), (-22 - 80, 1050 + 40), fill = '#5b5b5d')
	self.draw_rectangle ((-22, 500), (-22 - 80, 500 + 80), fill = '#5b5b5d')
	self.draw_rectangle ((-22, -80), (-22 - 80, 0), fill = '#5b5b5d')
	self.draw_rectangle ((-22, 0), (-22 - 80, 500), fill = '#5b5b5d')
	self.draw_rectangle ((-22 - 80 - 250, 0), (-22 - 80, 500), fill = '#6d6dad', stipple = 'gray75')
	self.draw_rectangle ((3000 + 22, 2100), (3000 + 22 + 80, 2100 + 80), fill = '#5b5b5d')
	self.draw_rectangle ((3000 + 22, 1050 - 40), (3000 + 22 + 80, 1050 + 40), fill = '#5b5b5d')
	self.draw_rectangle ((3000 + 22, 500), (3000 + 22 + 80, 500 + 80), fill = '#5b5b5d')
	self.draw_rectangle ((3000 + 22, -80), (3000 + 22 + 80, 0), fill = '#5b5b5d')
	self.draw_rectangle ((3000 + 22, 0), (3000 + 22 + 80, 500), fill = '#5b5b5d')
	self.draw_rectangle ((3000 + 22 + 80 + 250, 0), (3000 + 22 + 80, 500), fill = '#6d6dad', stipple = 'gray75')
	# Vertical dispensers.
	self.draw_rectangle ((-22, 2100 - 750 - 85 / 2), (0, 2100 - 750 + 85 / 2), fill = '#5b5b5b')
	self.draw_circle ((40, 2100 - 750), 40)
	self.draw_rectangle ((750 - 85 / 2, 2100), (750 + 85 / 2, 2100 + 22), fill = '#5b5b5b')
	self.draw_circle ((750, 2100 - 40), 40)
	self.draw_rectangle ((3000 + 22, 2100 - 750 - 85 / 2), (3000, 2100 - 750 + 85 / 2), fill = '#5b5b5b')
	self.draw_circle ((3000 - 40, 2100 - 750), 40)
	self.draw_rectangle ((3000 - 750 + 85 / 2, 2100), (3000 - 750 - 85 / 2, 2100 + 22), fill = '#5b5b5b')
	self.draw_circle ((3000 - 750, 2100 - 40), 40)
	# Horizontal dispenser.
	self.draw_rectangle ((3000 / 2 - 924 / 2, 2100 + 22), (3000 / 2 + 924 / 2, 2100 + 22 + 80 + 22), fill = '#5b5b5b')
	self.draw_rectangle ((3000 / 2 - 924 / 2 + 22, 2100 + 22), (3000 / 2 + 924 / 2 - 22, 2100 + 22 + 80), fill = '#5b5b5b')
	self.draw_rectangle ((3000 / 2 - 880 / 2 - 35 - 60, 2100), (3000 / 2 - 880 / 2 - 35, 2100 + 44), fill = '#5b5b5b')
	self.draw_rectangle ((3000 / 2 + 880 / 2 + 35 + 60, 2100), (3000 / 2 + 880 / 2 + 35, 2100 + 44), fill = '#5b5b5b')
	# Balls.
	balls = [ (800, 200, 'rb'), (800, 400, 'RB'), (800, 600, 'ww'),
		(1300, 200, 'rb'), (1300, 400, 'rb'), (1300, 600, 'ww'),
		(520, 800, 'WW'), (750, 40, 'RB'), (40, 750, 'WW'),
		(450, 1120, 'ww'), (750, 1070, 'ww'), (1050, 1020, 'ww'),
		(1500 - 72 / 2, -22 - 80 / 2, 'BR'),
		(1500 - 72 / 2 - 1 * 73, -22 - 80 / 2, 'RB'),
		(1500 - 72 / 2 - 2 * 73, -22 - 80 / 2, 'BR'),
		(1500 - 72 / 2 - 3 * 73, -22 - 80 / 2, 'RB'),
		(1500 - 72 / 2 - 4 * 73, -22 - 80 / 2, 'BR'),
		(1500 - 72 / 2 - 5 * 73, -22 - 80 / 2, 'RB'),
		(1500, 1000, 'W'),
		]
	balls_config = { 'r': { 'outline': '#bf4141' }, 'R': { 'fill': '#bf4141' },
		'b': { 'outline': '#4241bf' }, 'B': { 'fill': '#4241bf' },
		'w': { 'outline': '#bfbfbf' }, 'W': { 'fill': '#bfbfbf' } }
	for b in balls:
	    self.draw_circle ((3000 - b[0], 2100 - b[1]), 72 / 2,
		    **balls_config[b[2][0]])
	    if len (b[2]) > 1:
		self.draw_circle ((b[0], 2100 - b[1]), 72 / 2,
			**balls_config[b[2][1]])

class TableView (DrawableCanvas):
    """This class handle the view of the table and every items inside it."""

    WIDTH = 3000 + 2 * 22 + 2 * 80 + 2 * 250 + 2 * 10
    HEIGHT = 2100 + 2 * 22 + 2 * 80 + 2 * 10
    XORIGIN = -3000 / 2
    YORIGIN = -2100 / 2

    def __init__ (self, master = None):
	DrawableCanvas.__init__ (self, self.WIDTH, self.HEIGHT, self.XORIGIN,
		self.YORIGIN,
		master, borderwidth = 1, relief = 'sunken',
		background = 'white')
	self.table = Table (self)
	self.robot = Robot (self.table)
	self.robot.angle = pi / 3
	self.robot.pos = (700, 700)

    def draw (self):
	self.table.draw ()
	self.robot.draw ()

class Application (Frame):

    def __init__ (self, master = None):
        Frame.__init__ (self, master)
        self.pack (expand = 1, fill = 'both')
        self.createWidgets ()

    def createWidgets (self):
	self.quitButton = Button (self, text = 'Quit', command = self.quit)
	self.quitButton.pack (side = 'right', anchor = 'n')
	self.tableview = TableView (self)
	self.tableview.pack (expand = 1, fill = 'both')

app = Application()
app.mainloop()
