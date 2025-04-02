# scripts/oneCircle.py

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from simulation.simulation import Simulation
from clothes.Cloth import Cloth
from objects.Circle import Circle
from objects.Line import Line
from objects.Square import Square

def main():
    sim = Simulation("claw machine", dt=0.01, iterations=5, selfCollision=False)
    
    # TODO: add claw
    
    # add cloth
    cloth = Cloth(num=(10, 10), spacing=0.01, origin=(0, 0.5), KS=0.9, KC=1)
    sim.add_cloth(cloth)

    # add line
    line = Line(center=(0.0, -1.0), normal=(0,1), color=(0, 0, 0), drest=0.01)
    sim.add_object(line)
    
    # add squares
    squares = []
    squares.append(Square(center=(0.1, -0.8), size=(0.2, 0.4), color=(0, 0, 0), drest=0.01))
    squares.append(Square(center=(0.2, -0.5), size=(0.4, 0.2), color=(0, 0, 0), drest=0.01))
    squares.append(Square(center=(0.8, -0.5), size=(0.4, 0.2), color=(0, 0, 0), drest=0.01))
    squares.append(Square(center=(0.9, -0.8), size=(0.2, 0.4), color=(0, 0, 0), drest=0.01))
    
    for square in squares:
        sim.add_object(square)
    
    # sim.set_wind((1.0, 0.0))

    sim.run()

if __name__ == "__main__":
    main()
