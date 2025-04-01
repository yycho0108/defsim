# src/objects/circle.py

import numpy as np
from .object import Object

class Circle(Object):
    """
    Represents a circle for collision detection in 2D.
    """

    def __init__(self, center, radius, drest=0.001):
        self.center = np.array(center, dtype=np.float32)
        self.radius = radius
        self.drest = drest  # extra distance to allow small separation

    def solve_collision_constraint(self, p, old_p):
        cp = p - self.center
        dist = np.linalg.norm(cp)
        r = self.radius + self.drest

        if dist < r:
            # Move point outward along the circle normal
            n = cp / (dist + 1e-8)
            correction = (r - dist) * n
            return correction

        return np.array([0.0, 0.0], dtype=np.float32)
