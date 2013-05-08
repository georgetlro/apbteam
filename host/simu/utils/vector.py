# simu - Robot simulation. {{{
# encoding: utf-8
#
# Copyright (C) 2011 Nicolas Schodet
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
"""2D vector."""
import math

__all__ = ('vector')

class vector (object):
    """2D vector, with useful operators."""

    __slots__ = ('x', 'y')

    def __init__ (self, *xy):
        """Initialise the vector with given coordinates.

        >>> vector (1, 2)
        vector(1.0, 2.0)
        >>> vector ((1, 2))
        vector(1.0, 2.0)
        >>> vector (vector (1, 2))
        vector(1.0, 2.0)
        >>> vector ("hello")
        Traceback (most recent call last):
           ...
        TypeError
        >>> vector ("ab")
        Traceback (most recent call last):
           ...
        ValueError: invalid literal for float(): a
        """
        if len (xy) == 2:
            self.x, self.y = float (xy[0]), float (xy[1])
        elif len (xy) == 1:
            v = xy[0]
            if isinstance (v, vector):
                self.x, self.y = v.x, v.y
            elif len (v) == 2:
                self.x, self.y = float (v[0]), float (v[1])
            else:
                raise TypeError
        else:
            raise TypeError

    def __len__ (self):
        """Return vector length, always 2.

        >>> len (vector (1, 2))
        2
        """
        return 2

    def __repr__ (self):
        """Return text representation.

        >>> vector (1, 2)
        vector(1.0, 2.0)
        """
        return 'vector(%r, %r)' % (self.x, self.y)

    def __str__ (self):
        """Return informal text representation.

        >>> str (vector (1, 2))
        '(1.0, 2.0)'
        """
        return '(%s, %s)' % (self.x, self.y)

    def __getitem__ (self, index):
        """Return a coordinate by its index, for compatibility with
        sequences."""
        return (self.x, self.y)[index]

    def __eq__ (self, other):
        """Test for equality.

        >>> vector (1, 2) == vector (1, 3)
        False
        >>> vector (1, 2) == vector (2, 2)
        False
        >>> vector (1, 2) == vector (1, 2)
        True
        """
        return (self.x, self.y) == (other.x, other.y)

    def __ne__ (self, other):
        """Test for inequality.

        >>> vector (1, 2) != vector (1, 2)
        False
        """
        return not (self == other)

    def __cmp__ (self, other):
        """No comparison apart from equality test.

        >>> vector (1, 2) < vector (1, 2)
        Traceback (most recent call last):
           ...
        TypeError
        """
        raise TypeError

    def __add__ (self, other):
        """Addition.

        >>> vector (1, 2) + vector (3, 4)
        vector(4.0, 6.0)
        """
        return vector (self.x + other.x, self.y + other.y)

    def __sub__ (self, other):
        """Subtraction.

        >>> vector (1, 2) - vector (3, 4)
        vector(-2.0, -2.0)
        """
        return vector (self.x - other.x, self.y - other.y)

    def __mul__ (self, other):
        """Multiplication or dot product.

        >>> vector (1, 2) * 2
        vector(2.0, 4.0)
        >>> 3 * vector (1, 2)
        vector(3.0, 6.0)
        >>> vector (1, 2) * vector (3, 4)
        11.0
        """
        if isinstance (other, vector):
            return self.x * other.x + self.y * other.y
        else:
            return vector (self.x * other, self.y * other)
    __rmul__ = __mul__

    def __div__ (self, other):
        """Division.

        >>> vector (1, 2) / 2
        vector(0.5, 1.0)
        >>> vector (1, 2) / vector (3, 4)
        Traceback (most recent call last):
           ...
        TypeError: unsupported operand type(s) for /: 'float' and 'vector'
        >>> 3.0 / vector (1, 2)
        Traceback (most recent call last):
           ...
        TypeError: unsupported operand type(s) for /: 'float' and 'vector'
        """
        return vector (self.x / other, self.y / other)
    __truediv__ = __div__

    def __neg__ (self):
        """Negate.

        >>> -vector (1.0, 2.0)
        vector(-1.0, -2.0)
        """
        return vector (-self.x, -self.y)

    def __pos__ (self):
        """No-op.

        >>> +vector (1.0, 2.0)
        vector(1.0, 2.0)
        """
        return self

    def __abs__ (self):
        """Norm.

        >>> abs (vector (3.0, 4.0))
        5.0
        >>> vector (3.0, 4.0).norm ()
        5.0
        """
        return math.hypot (self.x, self.y)
    norm = __abs__

    def norm_squared (self):
        """Norm squared.

        >>> vector (3.0, 4.0).norm_squared ()
        25.0
        """
        return self.x ** 2 + self.y ** 2

    def unit (self):
        """Return normalized vector.

        >>> print vector (3.0, 4.0).unit ()
        (0.6, 0.8)
        """
        norm = math.hypot (self.x, self.y)
        return vector (self.x / norm, self.y / norm)

    def normal (self):
        """Return the vector rotated by pi/2.

        >>> vector (1, 2).normal ()
        vector(-2.0, 1.0)
        """
        return vector (-self.y, self.x)

    def rotate (self, angle):
        """Return the vector, rotated by given angle.

        >>> vector (1, 2).rotate (math.pi / 2) # doctest: +ELLIPSIS
        vector(-2.0, 1.0...)
        """
        c = math.cos (angle)
        s = math.sin (angle)
        return vector (c * self.x - s * self.y, s * self.x + c * self.y)

    def angle (self):
        """Return angle from (1, 0).

        >>> vector (1, 1).angle ()
        0.78539816339744828
        >>> vector (0, -1).angle ()
        -1.5707963267948966
        >>> vector (-1, -1).angle ()
        -2.3561944901923448
        """
        return math.atan2 (self.y, self.x)

    @staticmethod
    def polar (angle, norm):
        """Return a vector constructed from polar coordinates (angle, norm).

        >>> print vector.polar (math.pi / 4, math.sqrt (2))
        (1.0, 1.0)
        """
        return vector (norm * math.cos (angle), norm * math.sin (angle))

if __name__ == '__main__':
    def _test ():
        import doctest
        doctest.testmod ()
    def _perf_test ():
        import timeit
        def do (title, stmt, setup = None):
            n = 10000
            if setup is None:
                setup = 'import vector; v = vector.vector (1.2, 3.4)'
            t = timeit.timeit (stmt, setup, number = n)
            print title, '%.3f µs' % (1e6 * t / n)
        do ('init from floats', 'v = vector.vector (1.2, 3.4)')
        do ('init from tuple', 'v = vector.vector ((1.2, 3.4))')
        do ('init from vector', 'v2 = vector.vector (v)')
        do ('init from polar', 'v = vector.vector.polar (1.0, 1.0)')
        do ('abs', 'abs (v)')
        do ('norm_squared', 'v.norm_squared()')
        do ('unit', 'v.unit()')
    _test()
    _perf_test ()
