# src/objects/Box.py

import numpy as np
from pyglet import shapes
from .object import Object

class Square(Object):
    """
    inits a Square
        center : tuple (x, y)
            center of the square
        size : tuple (width, height)
            contains x size, y size of the square
        color : tuple(r, g, b) , optional
            rgb color of the square
    """
    def __init__(self, center, size, color=(0, 0, 0)):
        self.center = np.array(center, dtype=np.float32)
        self.size = size
        self.color = color

    """
    @OVERRIDE
    draws the object
        scene: pyglet.graphics.Batch
            the scene object
    """
    def draw(self, scene, scale, offset):
        cx, cy = self.center
        half_w, half_h = self.size[0]/2, self.size[1]/2
        
        if not hasattr(self, 'square_shape'):
            c255 = tuple(int(c * 255) for c in self.color)
            self.square_shape = shapes.Rectangle(
                x = (cx - half_w) * scale + offset,
                y = (cy - half_h) * scale + offset,
                width = self.size[0] * scale,
                height = self.size[1] * scale,
                color = c255,
                batch = scene
            )
        else:
            self.square_shape.x = (cx - half_w) * scale + offset
            self.square_shape.y = (cy - half_h) * scale + offset

    """
    @OVERRIDE
    solves the collision constraint for a point p
        p: numpy array [x, y]
            the point which collides
    """
    def solve_collision_constraint(self, p, radius=0.0):
        cp = p - self.center  # Vector from square center to particle
        half_width = self.size[0] / 2
        half_height = self.size[1] / 2
        
        # Find closest point on rectangle border
        # First, determine region
        inside_x = abs(cp[0]) < half_width
        inside_y = abs(cp[1]) < half_height
        
        if inside_x and inside_y:
            # Point is inside rectangle, find closest edge
            dx_right = half_width - cp[0]
            dx_left = cp[0] + half_width
            dy_top = half_height - cp[1]
            dy_bottom = cp[1] + half_height
            
            # Find minimum distance to edge
            min_dist = min(dx_right, dx_left, dy_top, dy_bottom)
            
            # Create correction vector along appropriate axis
            if min_dist == dx_right:
                return np.array([radius + min_dist, 0.0], dtype=np.float32)
            elif min_dist == dx_left:
                return np.array([-radius - min_dist, 0.0], dtype=np.float32)
            elif min_dist == dy_top:
                return np.array([0.0, radius + min_dist], dtype=np.float32)
            else:  # dy_bottom
                return np.array([0.0, -radius - min_dist], dtype=np.float32)
        else:
            # Find closest point on rectangle
            closest_x = max(-half_width, min(cp[0], half_width))
            closest_y = max(-half_height, min(cp[1], half_height))
            
            # Vector from closest point to circle center
            closest_vector = cp - np.array([closest_x, closest_y])
            dist = np.linalg.norm(closest_vector)
            
            # If distance is less than radius, we have a collision
            if dist < radius:
                return (radius - dist) * (closest_vector / dist)
            else:
                return np.array([0.0, 0.0], dtype=np.float32)