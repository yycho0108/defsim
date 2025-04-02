# scripts/twoCircles.py

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from simulation.Simulation import Simulation
from clothes.GridCloth import GridCloth
from objects.Square import Square
from objects.Circle import Circle
from objects.Line import Line

def main():
    sim = Simulation(name="test", res=(800, 600), dt=0.005, gravity=-9.8)

    # create a cloth
    cloth = Cloth(nx=32, ny=32, size_x=0.5, size_y=0.5, ks=0.95, kb=0.05, dt=0.005)
    sim.add_cloth(cloth)

    # optionally add a square in 2D
    s = Square(center=(0, -0.2), width=0.4, height=0.2, drest=0.01)
    sim.add_object(s)

    # add two circles
    c1 = Circle(center=(0.1, 0.2), radius=0.1)
    sim.add_object(c1)

    c2 = Circle(center=(0.0, -0.1), radius=0.1)
    sim.add_object(c2)

    # add line if needed
    line = Line(point=(0, -0.5), normal=(0, 1), drest=0.005)
    sim.add_object(line)

    sim.run()

if __name__ == "__main__":
    main()
