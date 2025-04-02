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
    def __init__(self, name, gravity=-10, dt=0.005, window_size=1500, world_size=2.0, iterations=4, selfCollision=True):
        # simulation properties
        self.name = name
        self.GRAVITY = gravity
        self.DT = dt
        self.NUM_ITERATIONS = iterations
        self.WIND = np.array([0.0, 0.0], dtype=np.float32)
        self.selfCollision = selfCollision

        # window properties
        self.world_size = world_size
        self.scale = window_size / world_size
        self.offset = window_size // 2
        self.window = pyglet.window.Window(window_size, window_size, self.name)
        pyglet.gl.glClearColor(1,1,1,1)

        # objects on the scene
        self.def_object = None
        self.objects = []
        
        # Create a Batch to draw the scene
        self.scene = pyglet.graphics.Batch()

        # bind on_draw
        @self.window.event
        def on_draw():
            self.on_draw()

    """
    adds object to the scene
        obj: Object
    """
    def add_object(self, obj):
        self.objects.append(obj)
    
    def set_wind(self, force):
        self.WIND = np.array(force, dtype=np.float32)

    """
    set deformable object to the scene
        obj: DefObject
    """
    def set_def_object(self, obj):
        self.def_object = obj

    """
    CORE PDB algorithm
    """
    def update(self, dt):
        self.def_object.external_forces(self.GRAVITY, self.WIND, self.DT)
        
        self.def_object.make_predictions(self.DT)
        
        for i in range(self.NUM_ITERATIONS):
            for o in self.objects:
                self.def_object.solve_collision_constraints(o)
            self.def_object.solve_stretching_constraint(self.NUM_ITERATIONS)
            # def_object.solve_bending_constraints(self.NUM_ITERATIONS)
            if self.selfCollision:
                self.def_object.solve_self_collision_constraints(self.NUM_ITERATIONS)
                
        self.def_object.apply_correction(self.DT)

    def on_draw(self):
        self.window.clear()

        # Draw all objects
        for obj in self.objects:
            obj.draw(self.scene, self.scale, self.offset)

        # Draw def object
        self.def_object.draw(self.scene, self.scale, self.offset)
            
        # Render the complete scene
        self.scene.draw()

    def run(self):
        # Schedule the update function
        pyglet.clock.schedule_interval(self.update, 1.0/60.0)
        # Start the Pyglet application loop
        pyglet.app.run()
