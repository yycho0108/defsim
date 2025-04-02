# scripts/single_obj.py

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from simulation.Simulation import Simulation
from defobjects.DefObject import DefObject
from objects.Line import Line
from objects.Square import Square
from objects.Circle import Circle

def main():
    sim = Simulation("single object", dt=0.001, iterations=5, selfCollision=True)
    
    # add def_obj
    def_obj = DefObject(num=(5, 5), spacing=0.05, origin=(0.0, 0.0), KS=1.0, KC=1.0)
    sim.add_def_object(def_obj)

    # add line
    line = Line(center=(0.0, -0.5), normal=(0,1), color=(0.3, 0.3, 0.3))
    sim.add_object(line)
    
    # add square
    # square = Square(center=(0.0, -0.5), size=(0.5, 0.1), color=(0.3, 0.3, 0.3))
    # sim.add_object(square)
    
    # add circle
    # circle = Circle(center=(0.0, -0.5), radius=0.1, color=(0.3, 0.3, 0.3))
    # sim.add_object(circle)
    
    sim.set_wind((1.0, 0.0))

    sim.run()

if __name__ == "__main__":
    main()
