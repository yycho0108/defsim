# src/objects/Plane.py

import numpy as np
import pyglet
from pyglet import shapes
from objects.Object import Object

class Plane(Object):
    """
    2D plane -> a line in 2D
    """

    """
    init Plaane
        center: numpy array [x, y]
            center of the plane
        normal: numpy array [x, y]
            normal of the plane
        color: tuple(r, g, b), optional
            rgb color of the circle, range[0-1]
    """
    def __init__(self, center, normal=(0,1), color=(1,1,1), drest=0.001):
        self.center = np.array(center, dtype=np.float32)
        self.normal = np.array(normal, dtype=np.float32)
        self.normal /= (np.linalg.norm(self.normal) + 1e-8)
        self.color = color
        self.drest = drest

    """
    @OVERRIDE
    draws the object
        scene: pyglet.graphics.Batch
            the scene object
    """
    def draw(self, scene):
        scale = 300.0
        offset_x = 400.0
        offset_y = 300.0

        r255 = tuple(int(c * 255) for c in self.color)

        # create a big line along the plane
        # plane line param:
        # We want a line passing center and perpendicular to normal
        # direction vector can be (normal.y, -normal.x)
        dir_v = np.array([self.normal[1], -self.normal[0]], dtype=np.float32)
        length = 2000.0  # large enough
        half = length * 0.5

        p_mid = self.center
        p1 = p_mid + dir_v * half
        p2 = p_mid - dir_v * half

        x1, y1 = p1
        x2, y2 = p2

        line_shape = shapes.Line(
            x=x1 * scale + offset_x,
            y=y1 * scale + offset_y,
            x2=x2 * scale + offset_x,
            y2=y2 * scale + offset_y,
            width=2,
            color=r255,
            batch=scene
        )

    """
    @OVERRIDE
    solves the collision constraint for a point p
        p: numpy array [x, y]
            the point which collides
    """
    def solve_collision_constraint(self, p, x):
        # plane eq: (p - center).dot(normal) = 0
        cp = p - self.center
        pen = np.dot(cp, self.normal)

        if pen < self.drest:
            correction = (self.drest - pen) * self.normal
            return correction
        return np.array([0.0, 0.0], dtype=np.float32)