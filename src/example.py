import pycartesian_franka as franka
import math

r = franka.Robot('176.16.0.1')
r.translate([0.1, 0, 0], 2)
r.rotate([math.pi/4, 0, 0])