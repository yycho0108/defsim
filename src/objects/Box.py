# src/objects/Box.py

import numpy as np
from objects.Object import Object

class Box(Object):
    """
    Represents an axis-aligned rectangle for collision detection in 2D.
    """

    def __init__(self, center, width, height, drest=0.001):
        self.center = np.array(center, dtype=np.float32)
        self.half_w = width / 2
        self.half_h = height / 2
        self.drest = drest

    def solve_collision_constraint(self, p, x):
        # Convert p to box local space where box center = (0,0)
        local_p = p - self.center

        # Check x-axis
        correction = np.array([0.0, 0.0], dtype=np.float32)

        left_limit = -self.half_w - self.drest
        right_limit = self.half_w + self.drest
        if local_p[0] < left_limit:
            correction[0] = left_limit - local_p[0]
        elif local_p[0] > right_limit:
            correction[0] = right_limit - local_p[0]

        # Check y-axis
        bottom_limit = -self.half_h - self.drest
        top_limit = self.half_h + self.drest
        if local_p[1] < bottom_limit:
            correction[1] = bottom_limit - local_p[1]
        elif local_p[1] > top_limit:
            correction[1] = top_limit - local_p[1]

        return correction
