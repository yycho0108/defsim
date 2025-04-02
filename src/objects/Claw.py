import numpy as np
from pyglet import shapes
from .Circle import Circle
from .Square import Square
from .Line import Line

class Claw(Square):
    def __init__(self, center, size, color=(0, 0, 0)):
        super().__init__(center, size, color)
        self.active = False
        self.border_color = (0, 0, 0)

    """
    @OVERRIDE
    draws the object
        scene: pyglet.graphics.Batch
            the scene object
    """
    def draw(self, scene, scale, offset):
        cx, cy = self.center
        half_w, half_h = self.size[0]/2, self.size[1]/2
        c255 = tuple(int(c * 255) for c in self.color)
        b255 = tuple(int(c * 255) for c in self.border_color)
        
        if not hasattr(self, 'square_shape'):
            self.border_shape = shapes.Rectangle(
                x = (cx - half_w) * scale + offset - 10,
                y = (cy - half_h) * scale + offset - 10,
                width = self.size[0] * scale + 20,
                height = self.size[1] * scale + 20,
                color = b255,
                batch = scene
            )
            self.square_shape = shapes.Rectangle(
                x = (cx - half_w) * scale + offset,
                y = (cy - half_h) * scale + offset,
                width = self.size[0] * scale,
                height = self.size[1] * scale,
                color = c255,
                batch = scene
            )
            self.line_shape = shapes.Line(
                x = cx * scale + offset,
                y = cy * scale + offset,
                x2 = cx * scale + offset,
                y2 = (cy + 2) * scale + offset,
                width = 10,
                color = c255,
                batch = scene
            )
        else:
            self.border_shape.x = (cx - half_w) * scale + offset - 10
            self.border_shape.y = (cy - half_h) * scale + offset - 10
            self.border_shape.color = b255
            
            self.square_shape.x = (cx - half_w) * scale + offset
            self.square_shape.y = (cy - half_h) * scale + offset
            self.square_shape.color = c255
            
            self.line_shape.x = cx * scale + offset
            self.line_shape.y = cy * scale + offset
            self.line_shape.x2 = cx * scale + offset
            self.line_shape.y2 = (cy + 2) * scale + offset
            self.line_shape.color = c255

    def move(self, dx, dy):
        self.center[0] += dx
        self.center[1] += dy

    def toggle_active(self):
        self.active = not self.active
        self.border_color = (0, 1, 0) if self.active else (0, 0, 0)
        
    # Just to prevent claw collide with objects (PBD X)
    def check_claw_collision(self, object, cx, cy):
        claw_left = cx - self.size[0] / 2
        claw_right = cx + self.size[0] / 2
        claw_bottom = cy - self.size[1] / 2
        claw_top = cy + self.size[1] / 2
        
        if isinstance(object, Square):
            rect_left = object.center[0] - object.size[0] / 2
            rect_right = object.center[0] + object.size[0] / 2
            rect_bottom = object.center[1] - object.size[1] / 2
            rect_top = object.center[1] + object.size[1] / 2
            return not (claw_left > rect_right or claw_right < rect_left or claw_bottom > rect_top or claw_top < rect_bottom)
        
        elif isinstance(object, Circle):
            circle_x, circle_y = object.center
            radius = object.radius
            
            closest_x = max(claw_left, min(circle_x, claw_right))
            closest_y = max(claw_bottom, min(circle_y, claw_top))
            
            distance_x = circle_x - closest_x
            distance_y = circle_y - closest_y
            distance_squared = distance_x ** 2 + distance_y ** 2
            
            return distance_squared <= radius ** 2
        
        elif isinstance(object, Line):
            corners = [
                (claw_left, claw_bottom),
                (claw_left, claw_top),
                (claw_right, claw_bottom),
                (claw_right, claw_top)
            ]
            a, b = object.normal
            c = -(object.center[0] * a + object.center[1] * b)
            
            signs = [np.sign(a * x + b * y + c) for x, y in corners]
            if all(s == signs[0] for s in signs):
                return False
            return True
        
        else:
            raise NotImplementedError("Collision check for this object type is not implemented.")