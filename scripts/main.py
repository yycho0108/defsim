# scripts/main.py

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from simulation import Simulation
from clothes.grid_cloth import GridCloth
from objects.Circle import Circle
from objects.Plane import Plane

def main():
    # create collision objects
    circle = Circle(center=(0.0, 0.3), radius=0.2, drest=0.005)
    plane = Plane(point=(0, -0.2), normal=(0, 1), drest=0.005)

    # create cloth
    cloth = Cloth(
        nx=20,
        ny=10,
        size_x=0.8,
        size_y=0.5,
        gravity=-9.8,
        damping=0.99,
        ks=0.6,
        kb=0.0,
        dt=0.01,
        collision_objects=[circle, plane]
    )

    # create simulation
    sim = Simulation(cloth)
    sim.run()

if __name__ == "__main__":
    main()
