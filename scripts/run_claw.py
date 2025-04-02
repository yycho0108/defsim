# scripts/oneCircle.py

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from simulation.ClawSimulation import ClawSimulation
from defobjects.DefObject import DefObject
from objects.Line import Line
from objects.Square import Square
from objects.Claw import Claw

def main():
    sim = ClawSimulation("claw machine", dt=0.01, iterations=10, selfCollision=False)
    
    # TODO: add claw
    claw = Claw(center=(-0.5, 0.0), size=(0.3, 0.1), color=(0.5, 0.5, 0.5))
    sim.add_claw(claw)
    
    # add doll
    doll = DefObject(num=(10, 10), spacing=0.025, origin=(-0.5, -0.8), KS=1.0, KC=1.0)
    sim.add_def_object(doll)

    # add lines (walls)
    lines = []
    lines.append(Line(center=(0.0, -1.0), normal=(0,1), color=(1, 0, 0), drest=0.01))
    lines.append(Line(center=(-1.0, 0.0), normal=(1,0), color=(1, 0, 0), drest=0.01))
    lines.append(Line(center=(1.0, 0.0), normal=(-1,0), color=(1, 0, 0), drest=0.01))
    for line in lines:
        sim.add_object(line)
    
    # add squares (gate)
    squares = []
    squares.append(Square(center=(0.1, -0.8), size=(0.2, 0.4), color=(0, 0, 1), drest=0.01))
    squares.append(Square(center=(0.2, -0.5), size=(0.4, 0.2), color=(0, 0, 1), drest=0.01))
    squares.append(Square(center=(0.8, -0.5), size=(0.4, 0.2), color=(0, 0, 1), drest=0.01))
    squares.append(Square(center=(0.9, -0.8), size=(0.2, 0.4), color=(0, 0, 1), drest=0.01))
    for square in squares:
        sim.add_object(square)
    
    sim.set_wind((3.0, 0.0))
    
    sim.run()

if __name__ == "__main__":
    main()
