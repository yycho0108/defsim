# src/objects/Object.py

from abc import ABCMeta, abstractmethod

class Object(object, metaclass=ABCMeta):
    """
    Base 2D object class for collision handling in PBD.
    """

    @abstractmethod
    def __init__(self):
        pass
    
    """
    Draws the object in a pyglet Batch or Scene.
    scene: pyglet.graphics.Batch (or similar)
    """
    @abstractmethod
    def draw(self, scene):
        pass

    """
    Check if p collides with the object
    p: (numpy array) predicted position, shape (2,)
    x: (numpy array) old position, shape (2,)
    """
    @abstractmethod
    def solve_collision_constraint(self, p, x):
        pass
