# scripts/oneSphere.py

import sys
import os

# adjust sys.path for convenience
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.join(CURRENT_DIR, '..')
SRC_DIR = os.path.join(PARENT_DIR, 'src')
if SRC_DIR not in sys.path:
    sys.path.append(SRC_DIR)

from simulation import Simulation
from clothes.cloth import Cloth
from objects.circle import Circle
from objects.plane import Plane

def main():
    # create Simulation
    sim = Simulation(name="OneSphereDemo", res=(800, 600), dt=0.01, gravity=-9.8)

    # add a cloth
    cloth = Cloth(nx=32, ny=32, size_x=0.5, size_y=0.5, gravity=-9.8, dt=0.01)
    sim.add_cloth(cloth)

    # add a sphere
    s = Circle(center=(0.0, 0.3), radius=0.1, drest=0.005)
    sim.add_object(s)

    # add a plane (as floor)
    p = Plane(point=(0, -0.3), normal=(0, 1), drest=0.005)
    sim.add_object(p)

    # run
    sim.run()

if __name__ == "__main__":
    main()
