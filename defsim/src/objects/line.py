# src/objects/Line.py

import numpy as np
from pyglet import shapes
from .object import Object

class Line(Object):
    """
    init Plane
        center: tuple (x, y)
            center of the surface of half plane
        normal: tuple (x, y)
            normal of the line
        color: tuple(r, g, b), optional
            rgb color of the circle, range[0-1]
    """
    def __init__(self, center, normal=(0,1), color=(0, 0, 0)):
        self.center = np.array(center, dtype=np.float32)
        self.normal = np.array(normal, dtype=np.float32)
        self.normal = self.normal / np.linalg.norm(self.normal)
        self.color = color

    """
    @OVERRIDE
    draws the object
        scene: pyglet.graphics.Batch
            the scene object
    """
    def draw(self, scene, scale, offset):
        perp = np.array([self.normal[1], -self.normal[0]], dtype=np.float32)
        p1 = self.center + perp * 1000
        p2 = self.center - perp * 1000
        
        if not hasattr(self, 'line_shape'):
            c255 = tuple(int(c * 255) for c in self.color)
            self.line_shape = shapes.Line(
                x = p1[0] * scale + offset,
                y = p1[1] * scale + offset,
                x2 = p2[0] * scale + offset,
                y2 = p2[1] * scale + offset,
                width = 10,
                color = c255,
                batch = scene
            )
        else:
            self.line_shape.x = p1[0] * scale + offset
            self.line_shape.y = p1[1] * scale + offset
            self.line_shape.x2 = p2[0] * scale + offset
            self.line_shape.y2 = p2[1] * scale + offset

    """
    @OVERRIDE
    solves the collision constraint for a point p
        p: numpy array [x, y]
            the point which collides
    """        
    def solve_collision_constraint(self, p, radius=0.0):
        cp = p - self.center
        signed_dist = np.dot(cp, self.normal)
        
        if signed_dist < radius:
            penetration = radius - signed_dist
            return penetration * self.normal
        else:
            return np.array([0.0, 0.0], dtype=np.float32)