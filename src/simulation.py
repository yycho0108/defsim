# src/simulation/simulation.py

import numpy as np
import pyglet
from pyglet.gl import GL_LINES

class Simulation:
    """
    new simulation
        name: string
            name of the simulation
        gravity: float
            gravity < 0
        dt: float
            time step size used in the simulation
        res: tuple(width, height)
            window size in pixels
    """
    def __init__(self, name, gravity=-10, dt=0.01, res=(800,600), iterations=4, selfCollision=False):
        # simulation properties
        self.name = name
        self.GRAVITY = gravity
        self.DT = dt
        self.NUM_ITERATIONS = iterations
        self.WIND = np.array([0.0, 0.0], dtype=np.float32)
        self.selfCollision = selfCollision
        
        # window properties
        self.WIDTH, self.HEIGHT = res
        self.window = pyglet.window.Window(self.WIDTH, self.HEIGHT, self.name)

        # objects of the scene
        self.lights = []
        self.cloths = []
        self.objects = []
        
        # we'll do our rendering in on_draw callback
        @self.window.event
        def on_draw():
            self.on_draw()

        # scene update scheduling
        pyglet.clock.schedule_interval(self.update, 1.0/60.0)
        
    """
    add new light to the scene
        light_pos : tuple(x, y, z)
            position of the light
        light_color : tuple(r, g, b) 
            color of the light, in range [0, 1]
    """
    def add_light(self, pos, color=(1,1,1)):
        self.lights.append((pos, color))
        
    """
    adds object to the scene
        obj: Object
            scene object Object.py
    """
    def add_object(self, obj):
        self.objects.append(obj)
        
    def set_wind(self, force):
        self.WIND = force
        
    """
    adds object to the scene
        cloth: Cloth
            cloth Cloth.py
    """
    def add_cloth(self, cloth):
        self.cloths.append(cloth)

    """
    CORE PBD algorithm
    """
    def update_cloth(self, cloth):
        cloth.external_forces(self.GRAVITY, self.WIND, self.DT)
        cloth.step(collision_objects=self.objects)

    def update(self, dt):
        for c in self.cloths:
            self.update_cloth(c)
    
        def on_draw(self):
            self.window.clear()
            
        scene = pyglet.graphics.Batch()
        
        for l in self.lights:
            light = pyglet.shapes.Circle(
                x=l[0][0], y=l[0][1], radius=5,
                color=(255, 255, 0), batch=scene
            )
            light.draw()

        for o in self.objects:
            o.draw(scene)

        for c in self.cloths:
            c.draw(scene)

        scene.draw()

    """
    runs the simulation
    """
    def run(self):
        pyglet.app.run()
