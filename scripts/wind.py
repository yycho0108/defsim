# scripts/wind.py

import sys
import os

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.join(CURRENT_DIR, '..')
SRC_DIR = os.path.join(PARENT_DIR, 'src')
if SRC_DIR not in sys.path:
    sys.path.append(SRC_DIR)

from simulation import Simulation
from clothes.cloth import Cloth
from objects.plane import Plane

def main():
    sim = Simulation(name="WindDemo", res=(800, 600), dt=0.01, gravity=-0.9)

    # set global wind
    sim.set_wind((2.0, 1.0))  # blowing to the right-up

    # create cloth
    cloth = Cloth(nx=32, ny=32, size_x=0.5, size_y=0.5, gravity=-0.9, dt=0.01)
    # fix some points if you want
    cloth.fix_point(0)
    cloth.fix_point(31 * 32)  # e.g. top-right corner in row-major indexing

    sim.add_cloth(cloth)

    # add plane (like ground)
    plane = Plane(point=(0, -0.3), normal=(0, 1), drest=0.01)
    sim.add_object(plane)

    sim.run()

if __name__ == "__main__":
    main()
