# src/objects/Box.py

import numpy as np
import pyglet
from pyglet import shapes
from .Object import Object

def make_rotation_matrix(angle_radians):
    """
    Returns a 2x2 rotation matrix for given angle in radians.
    R = [[ cosθ, -sinθ ],
         [ sinθ,  cosθ ]]
    """
    c = np.cos(angle_radians)
    s = np.sin(angle_radians)
    return np.array([[c, -s],
                     [s,  c]], dtype=np.float32)

class Square(Object):
    """
    inits a Square
        center : tuple (x, y)
            center of the square
        size : tuple (width, height)
            contains x size, y size of the square
        color : tuple(r, g, b) , optional
            rgb color of the square
        rotation : float
            counter clockwise rotation in degrees
    """
    def __init__(self, center, size, color=(1,1,1), drest=0.01, rotation=0.0):
        self.center = np.array(center, dtype=np.float32)
        self.size = size
        self.color = color
        self.drest = drest
        
        self.rotation = rotation
        self.R = make_rotation_matrix(self.rotation)

    """
    @OVERRIDE
    draws the object
        scene: pyglet.graphics.Batch
            the scene object
    """
    def draw(self, scene, scale):
        cx, cy = self.center
        half_w, half_h = self.size[0]/2, self.size[1]/2
        
        if not hasattr(self, 'square_shape'):
            c255 = tuple(int(c*255) for c in self.color)
            self.square_shape = shapes.Rectangle(
                x=(cx - half_w)*scale,
                y=(cy - half_h)*scale,
                width=self.size[0]*scale,
                height=self.size[1]*scale,
                color=c255,
                batch=scene
            )
            self.square_shape.rotation = -self.rotation
        else:
            self.square_shape.x = (cx - half_w)*scale
            self.square_shape.y = (cy - half_h)*scale

    """
    @OVERRIDE
    solves the collision constraint for a point p
        p: numpy array [x, y]
            the point which collides
    """
    def solve_collision_constraint(self, p, x):
        cp = p - self.center
        w = self.R @ np.array([self.size[0]/2, 0])
        h = self.R @ np.array([0, self.size[1]/2])
        what = w / np.linalg.norm(w)
        hhat = h / np.linalg.norm(h)
        
        dw = cp.dot(what)
        dh = cp.dot(hhat)
        
        absdw = np.linalg.norm(w) + self.drest - abs(dw)
        absdh = np.linalg.norm(h) + self.drest - abs(dh)
        
        res = 0
        if absdw > 0 and absdh > 0:
            argmin = 1
            vmin = absdw
            if vmin > absdh:
                argmin = 2
                vmin = absdh
            
            if argmin == 1:
                res = what * (absdw)
                if dw < 0:
                    res = -res
            elif argmin == 2:
                res = hhat * (absdh)
                if dh < 0:
                    res = -res
        return res