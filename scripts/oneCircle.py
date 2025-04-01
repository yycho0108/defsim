# scripts/oneCircle.py

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from simulation.simulation import Simulation
from clothes.GridCloth import GridCloth
from objects.Circle import Circle
from objects.Line import Line
from objects.Square import Square

def main():
    sim = Simulation("test", dt=0.01, iterations=3)
    
    # add cloth
    cloth = GridCloth(N=(32, 32), S=(0.5, 0.5), KS=1, KB=0.2, KC=1, center=(0, 0, 0.01))
    sim.add_object(cloth)

    # add circle
    circle = Circle(center=(0.0, 0.3), radius=0.1, color=(1,0,0), drest=0.005)
    sim.add_object(circle)

    # add line
    line = Line(center=(0.0, 0.1), normal=(0,1), color=(0.3, 0.3, 0.3), drest=0.01)
    sim.add_object(line)

    sim.run()

if __name__ == "__main__":
    main()
