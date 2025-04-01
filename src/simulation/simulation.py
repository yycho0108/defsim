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
    def __init__(self, name, gravity=-10, dt=0.005, res=(800,600), iterations=4, selfCollision=True):
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

        # objects on the scene
        self.lights = []
        self.cloths = []
        self.objects = []

        # bind on_draw
        @self.window.event
        def on_draw():
            self.on_draw()

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
        self.WIND = np.array(force, dtype=np.float32)

    """
    adds object to the scene
        cloth: Cloth
            cloth Cloth.py
    """
    def add_cloth(self, cloth):
        self.cloths.append(cloth)

    """
    CORE PDB algorithm
    """
    def update_cloth(self, c):
        c.external_forces(self.GRAVITY, self.WIND, self.DT)
        
        c.make_predictions(self.DT)
        
        for i in range(self.NUM_ITERATIONS):
            for o in self.objects:
                c.solve_collision_constraints(o)
            c.solve_stretch_constraints(self.NUM_ITERATIONS)
            c.solve_bending_constraints(self.NUM_ITERATIONS)
            if self.selfCollision:
                c.solve_self_collision_constraints(self.NUM_ITERATIONS)
                
        c.apply_correction(self.DT)

    def update(self, dt):
        # Update each cloth object
        for c in self.cloths:
            self.update_cloth(c)

        # Redraw the scene in the next frame
        self.window.dispatch_event('on_draw')

    def on_draw(self):
        self.window.clear()

        # Create a Batch to draw the scene
        scene = pyglet.graphics.Batch()

        # Draw lights as small circles
        for lpos, lcolor in self.lights:
            col255 = (int(lcolor[0]*255), int(lcolor[1]*255), int(lcolor[2]*255))
            light_circle = pyglet.shapes.Circle(
                x=int(lpos[0] * self.WIDTH), y=int(lpos[1] * self.HEIGHT), radius=5,
                color=col255, batch=scene
            )

        # Draw all objects
        for obj in self.objects:
            obj.draw(scene)

        # Draw all cloths
        for cloth in self.cloths:
            cloth.draw(scene)

        # Render the complete scene
        scene.draw()

    def run(self):
        # Schedule the update function
        pyglet.clock.schedule_interval(self.update, 1.0/60.0)
        # Start the Pyglet application loop
        pyglet.app.run()
