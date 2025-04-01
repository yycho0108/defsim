# src/objects/Circle.py

import numpy as np
import pyglet
from pyglet import shapes
from objects.Object import Object

class Circle(Object):
    """
    2D circle for collision detection.
    """
    
    """
    init Circle
        center: numpy array [x, y]
            center of the circle
        radius: float
            radius of the circle
        color: tuple(r, g, b), optional
            rgb color of the circle, range[0-1]
    """
    def __init__(self, center, radius, color=(1, 1, 1), drest=0.001):
        self.center = np.array(center, dtype=np.float32)
        self.radius = radius
        self.color = color
        self.drest = drest

    """
    @OVERRIDE
    draws the object
        scene: pyglet.graphics.Batch
            the scene object
    """
    def draw(self, scene):
        # scaling to visualize
        scale = 300.0
        offset_x = 400.0
        offset_y = 300.0

        x, y = self.center
        r = self.radius
        r255 = tuple(int(c * 255) for c in self.color)  # convert (0-1) to (0-255)

        # pyglet.shapes.Circle requires a batch if we want to group drawing
        circle_shape = shapes.Circle(
            x=x*scale + offset_x,
            y=y*scale + offset_y,
            radius=r*scale,
            color=r255,
            batch=scene
        )
        # Note: shapes.Circle will be drawn automatically when scene.draw() is called

    """
    @OVERRIDE
    solves the collision constraint for a point p
        p: numpy array [x, y]
            the point which collides
    """
    def solve_collision_constraint(self, p, x):
        cp = p - self.center
        dist = np.linalg.norm(cp)
        boundary = self.radius + self.drest

        if dist < boundary:
            # push out along the normal
            n = cp / (dist + 1e-8)
            correction = (boundary - dist) * n
            return correction
        return np.array([0.0, 0.0], dtype=np.float32)
