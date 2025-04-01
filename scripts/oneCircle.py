# scripts/oneCircle.py

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from simulation.simulation import Simulation
from clothes.Cloth import Cloth
from objects.Circle import Circle
from objects.Line import Line

def main():
    sim = Simulation("test", dt=0.02, iterations=5)
    
    # add cloth
    cloth = Cloth(num=(10, 10), spacing=0.05, origin=(0, 0.5), KS=0.9, KC=1)
    sim.add_cloth(cloth)

    # add circle
    circle = Circle(center=(0.0, -0.5), radius=0.1, color=(1,0,0), drest=0.001)
    sim.add_object(circle)

    # add line
    line = Line(center=(0.0, -0.5), normal=(0,1), color=(0.3, 0.3, 0.3), drest=0.001)
    sim.add_object(line)
    
    sim.set_wind((1.0, 0.0))

    sim.run()

if __name__ == "__main__":
    main()
