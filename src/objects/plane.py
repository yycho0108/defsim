# src/objects/plane.py

import numpy as np
from .object import Object

class Plane(Object):
    """
    Represents a plane (in 2D, it's a line) for collision detection.
    plane is defined by a point on the plane and a normal vector.
    """

    def __init__(self, point, normal, drest=0.001):
        self.point = np.array(point, dtype=np.float32)
        self.normal = np.array(normal, dtype=np.float32)
        norm_len = np.linalg.norm(self.normal) + 1e-8
        self.normal /= norm_len
        self.drest = drest

    def solve_collision_constraint(self, p, old_p):
        cp = p - self.point
        pen = np.dot(cp, self.normal)  # penetration distance

        if pen < self.drest:
            # Push out from the plane by the difference
            correction = (self.drest - pen) * self.normal
            return correction

        return np.array([0.0, 0.0], dtype=np.float32)
