# src/objects/object.py

import numpy as np

class Object():
    """
    Base 2D object class for collision handling in PBD.
    """

    def solve_collision_constraint(self, p, old_p):
        """
        Returns the correction vector for position p if it penetrates the object.
        p: predicted position (numpy array of shape (2,))
        old_p: previous position (numpy array of shape (2,))
        Returns: numpy array of shape (2,) for position correction
        """
        return np.array([0.0, 0.0], dtype=np.float32)
