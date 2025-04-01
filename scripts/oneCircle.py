# scripts/oneCircle.py

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from simulation.simulation import Simulation
from clothes.GridCloth import GridCloth
from objects.Circle import Circle
from objects.Line import Line

def main():
    sim = Simulation("test", res=(800,600), dt=0.01, iterations=3)

    # add cloth
    cloth = GridCloth(
        N=(32,32),
        cell_size=(0.5, 0.5),
        center=(0.0,0.0),
        gravity=-9.8,
        dt=0.01,
        wind=(0.0,0.0),
        color=(0.5,0.5,0.5),
        stiffness=1.0,
        damping=0.99
    )
    sim.add_cloth(cloth)

    # add circle
    circle = Circle(center=(0.0, 0.3), radius=0.1, color=(1,0,0), drest=0.005)
    sim.add_object(circle)

    # add line
    line = Line(center=(0.0, -0.3), normal=(0,1), color=(0.3, 0.3, 0.3), drest=0.01)
    sim.add_object(line)

    sim.run()

if __name__ == "__main__":
    main()
