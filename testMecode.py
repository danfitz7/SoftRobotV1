# -*- coding: utf-8 -*-
from mecode import G
from math import sqrt

import numpy as np
import matplotlib.pyplot as plt

g = G(
    print_lines=False,
    outfile=r"H:\User Files\Fitzgerald\SoftRobots\Parameterized\gcode\parameterizedValves.pgm",
    aerotech_include=False,
)

g.feed(20)
g.move(x=0, y=5)
g.feed(5)
g.move(x=5,y=5)
g.move(x=5, y=10)
g.abs_move(**{"z":10})
g.move(x=10, y=10, **{"z":2})

g.view()

g.teardown()