# src/simulation/simulation.py

import pyglet
from pyglet.gl import GL_LINES

class Simulation:
    """
    A flexible 2D PBD simulation class that can handle multiple cloths, objects, lights, etc.
    """

    def __init__(self, name="Simulation", res=(800, 600), dt=0.01, gravity=-9.8):
        self.name = name
        self.width, self.height = res
        self.dt = dt
        self.gravity = gravity

        # store cloths and objects
        self.cloths = []
        self.objects = []

        # create a pyglet window
        self.window = pyglet.window.Window(
            width=self.width,
            height=self.height,
            caption=self.name
        )

        @self.window.event
        def on_draw():
            self.window.clear()
            batch = pyglet.graphics.Batch()

            # draw cloths
            for cloth in self.cloths:
                cloth.draw(batch)

            # if objects themselves have some draw method, we could also call it here
            # e.g. for debugging or visuals

            batch.draw()

        # schedule the update
        pyglet.clock.schedule_interval(self.update, 1.0/60.0)

    def set_wind(self, wind):
        """
        Sets wind for all cloths. If you want different wind per cloth,
        call cloth.set_wind(...) individually.
        """
        for c in self.cloths:
            c.set_wind(wind)

    def add_cloth(self, cloth):
        self.cloths.append(cloth)

    def add_object(self, obj):
        """
        Register an object so all cloths can collide with it.
        """
        self.objects.append(obj)

    def update(self, dt):
        """
        Each frame, step cloths and handle collisions with objects.
        """
        for cloth in self.cloths:
            # update the cloth's collision object list from our simulation
            cloth.collision_objects = self.objects
            cloth.step(substeps=3)

    def run(self):
        pyglet.app.run()
