# scripts/run_sim.py

import sys
import os

# make sure we can import from src
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.join(CURRENT_DIR, '..')
SRC_DIR = os.path.join(PARENT_DIR, 'src')
if SRC_DIR not in sys.path:
    sys.path.append(SRC_DIR)

from objects.circle import Circle
from objects.plane import Plane
from clothes.cloth import Cloth
from simulation import Simulation

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
