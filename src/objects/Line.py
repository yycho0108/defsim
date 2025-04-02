# src/objects/Line.py

import numpy as np
import pyglet
from pyglet import shapes
from .Object import Object

class Line(Object):
    """
    init Plaane
        center: tuple (x, y)
            center of the line
        normal: tuple (x, y)
            normal of the line
        color: tuple(r, g, b), optional
            rgb color of the circle, range[0-1]
    """
    def __init__(self, center, normal=(0,1), L=1000, color=(0, 0, 0)):
        self.center = np.array(center, dtype=np.float32)
        self.normal = np.array(normal, dtype=np.float32)
        self.normal = self.normal / np.linalg.norm(self.normal)
        self.color = color
        self.L = L  # assume that line is infinitely long

    """
    @OVERRIDE
    draws the object
        scene: pyglet.graphics.Batch
            the scene object
    """
    def draw(self, scene, scale, offset):
        perp = np.array([self.normal[1], -self.normal[0]], dtype=np.float32)
        p1 = self.center + perp * (self.L * 0.5)
        p2 = self.center - perp * (self.L * 0.5)
        
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
    def solve_collision_constraint(self, p, x):
        cp = p - self.center
        
        l = abs(np.dot(cp, self.normal))
        if np.dot(cp, self.normal) < 0:
            return l * self.normal
        else:
            return np.array([0.0, 0.0], dtype=np.float32)