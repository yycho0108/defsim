# scripts/twoCircles.py

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from simulation import Simulation
from clothes.grid_cloth import GridCloth
from objects.Circle import Circle
from objects.Plane import Plane

def main():
    sim = Simulation(name="test", res=(800, 600), dt=0.005, gravity=-9.8)

    # create a cloth
    cloth = Cloth(nx=32, ny=32, size_x=0.5, size_y=0.5, ks=0.95, kb=0.05, dt=0.005)
    sim.add_cloth(cloth)

    # optionally add a box in 2D
    b = Box(center=(0, -0.2), width=0.4, height=0.2, drest=0.01)
    sim.add_object(b)

    # add two circles
    s1 = Circle(center=(0.1, 0.2), radius=0.1)
    sim.add_object(s1)

    s2 = Circle(center=(0.0, -0.1), radius=0.1)
    sim.add_object(s2)

    # add plane if needed
    plane = Plane(point=(0, -0.5), normal=(0, 1), drest=0.005)
    sim.add_object(plane)

    sim.run()

if __name__ == "__main__":
    main()
