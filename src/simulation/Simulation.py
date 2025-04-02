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
        self.def_objects = []
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
    adds object to the scene
        obj: DefObject
    """
    def add_def_object(self, obj):
        self.def_objects.append(obj)

    """
    CORE PDB algorithm
    """
    def update_def_obj(self, def_obj):
        def_obj.external_forces(self.GRAVITY, self.WIND, self.DT)
        
        def_obj.make_predictions(self.DT)
        
        for i in range(self.NUM_ITERATIONS):
            for o in self.objects:
                def_obj.solve_collision_constraints(o)
            def_obj.solve_stretching_constraint(self.NUM_ITERATIONS)
            # def_obj.solve_bending_constraints(self.NUM_ITERATIONS)
            if self.selfCollision:
                def_obj.solve_self_collision_constraints(self.NUM_ITERATIONS)
                
        def_obj.apply_correction(self.DT)

    def update(self, dt):
        # Update each def object
        for def_obj in self.def_objects:
            self.update_def_obj(def_obj)

    def on_draw(self):
        self.window.clear()

        # Draw all objects
        for obj in self.objects:
            obj.draw(self.scene, self.scale, self.offset)

        # Draw all def objects
        for def_obj in self.def_objects:
            def_obj.draw(self.scene, self.scale, self.offset)
            
        # Render the complete scene
        self.scene.draw()

    def run(self):
        # Schedule the update function
        pyglet.clock.schedule_interval(self.update, 1.0/60.0)
        # Start the Pyglet application loop
        pyglet.app.run()
