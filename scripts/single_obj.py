# scripts/single_obj.py

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from simulation.Simulation import Simulation
from defobjects.DefObject import DefObject
from objects.Circle import Circle
from objects.Line import Line

def main():
    sim = Simulation("test", dt=0.005, iterations=10, selfCollision=True)
    
    # add def_obj
    def_obj = DefObject(num=(5, 5), spacing=0.05, origin=(0, -0.3), KS=1.0, KC=1.0)
    sim.add_def_object(def_obj)

    # add line
    line = Line(center=(0.0, -0.5), normal=(0,1), color=(0.3, 0.3, 0.3))
    sim.add_object(line)

    sim.run()

if __name__ == "__main__":
    main()
