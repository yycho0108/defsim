# src/objects/Circle.py

import numpy as np
import pyglet
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
    def __init__(self, center, radius, color=(1, 1, 1), drest=0.01):
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
    def draw(self, scene, scale):
        x, y = self.center
        
        if not hasattr(self, 'circle_shape'):
            r255 = tuple(int(c * 255) for c in self.color)
            self.circle_shape = shapes.Circle(
                x=x*scale,
                y=y*scale,
                radius=self.radius*scale,
                color=r255,
                batch=scene
            )
        else:
            self.circle_shape.x = x * scale
            self.circle_shape.y = y * scale
            self.circle_shape.radius = self.radius * scale
    
    """
    @OVERRIDE
    solves the collision constraint for a point p
        p: numpy array [x, y]
            the point which collides
    """
    def solve_collision_constraint(self, p, x):
        cp = p - self.center
        res = 0
        if np.linalg.norm(cp) < (self.radius + self.drest):
            if np.linalg.norm(x - self.center) < (self.radius + self.drest):
                n = cp / np.linalg.norm(cp)
                res = (self.radius + self.drest - np.linalg.norm(cp)) * n
            
            else:
                d = p - x
                oc = x - self.center
                a = d.dot(d)
                b = 2.0 * oc.dot(d)
                c = oc.dot(oc) - (self.radius + self.drest) * (self.radius + self.drest)
                
                t = 0
                disc = b * b - 4 * a * c
                if disc > 0:
                    t = (-b - np.sqrt(disc)) / (2 * a)
                    
                collision_point = x + t * d
                n = collision_point - self.center
                n = n / np.linalg.norm(n)
                
                C = np.dot((p - collision_point), n) - self.drest
                res = - C * n
        return res
