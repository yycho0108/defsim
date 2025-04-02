# src/objects/Circle.py

import numpy as np
from pyglet import shapes
from .Object import Object

class Circle(Object):
    """
    init Circle
        center: tuple (x, y)
            center of the circle
        radius: float
            radius of the circle
        color: tuple(r, g, b), optional
            rgb color of the circle, range[0-1]
    """
    def __init__(self, center, radius, color=(0, 0, 0)):
        self.center = np.array(center, dtype=np.float32)
        self.radius = radius
        self.color = color

    """
    @OVERRIDE
    draws the object
        scene: pyglet.graphics.Batch
            the scene object
    """
    def draw(self, scene, scale, offset):
        x, y = self.center
        
        if not hasattr(self, 'circle_shape'):
            r255 = tuple(int(c * 255) for c in self.color)
            self.circle_shape = shapes.Circle(
                x = x * scale + offset,
                y = y * scale + offset,
                radius = self.radius * scale,
                color = r255,
                batch = scene
            )
        else:
            self.circle_shape.x = x * scale + offset
            self.circle_shape.y = y * scale + offset
    
    """
    @OVERRIDE
    solves the collision constraint for a point p
        p: numpy array [x, y]
            the point which collides
    """
    def solve_collision_constraint(self, p, radius=0.0):
        v = p - self.center
        dist = np.linalg.norm(v)
        
        combined_radius = self.radius + radius
        
        if dist < combined_radius:
            # If particle is exactly at the center, push arbitrarily to the right
            if dist < 1e-6:
                v = np.array([1.0, 0.0], dtype=np.float32)
                dist = 1.0
            
            # Push just enough to make surfaces touch (not centers)
            return (combined_radius - dist) * (v / dist)
        else:
            return np.array([0.0, 0.0], dtype=np.float32)