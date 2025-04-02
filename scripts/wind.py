# scripts/wind.py

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from simulation.Simulation import Simulation
from clothes.GridCloth import GridCloth
from objects.Circle import Circle
from objects.Line import Line

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

    # add line (like ground)
    line = Line(point=(0, -0.3), normal=(0, 1), drest=0.01)
    sim.add_object(line)

    sim.run()

if __name__ == "__main__":
    main()
