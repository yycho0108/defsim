import numpy as np
import pyglet
from pyglet import shapes
from pyglet.window import key
from .Simulation import Simulation

class ClawSimulation(Simulation):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        self.move_speed = 0.01
        self.attract_force = 10.0
        self.attract_radius = 1.0
        self.key_state = {key.W: False, key.S: False, key.A: False, key.D: False, key.UP: False, key.DOWN: False, key.LEFT: False, key.RIGHT: False}

        self.window.push_handlers(on_key_press=self.on_key_press)
        self.window.push_handlers(on_key_release=self.on_key_release)

    def add_claw(self, claw):
        self.add_object(claw)
        self.claw = claw

    def on_key_press(self, symbol, modifiers):
        if symbol in self.key_state:
            self.key_state[symbol] = True
        elif symbol == key.SPACE:
            self.claw.toggle_active()

    def on_key_release(self, symbol, modifiers):
        if symbol in self.key_state:
            self.key_state[symbol] = False

    def move_claw(self):
        dx, dy = 0, 0
        if self.key_state[key.W] or self.key_state[key.UP]:
            dy += self.move_speed
        if self.key_state[key.S] or self.key_state[key.DOWN]:
            dy -= self.move_speed
        if self.key_state[key.A] or self.key_state[key.LEFT]:
            dx -= self.move_speed
        if self.key_state[key.D] or self.key_state[key.RIGHT]:
            dx += self.move_speed
        
        # prevent the invalid move
        new_x = self.claw.center[0] + dx
        new_y = self.claw.center[1] + dy
            
        for obj in self.objects:
            if obj == self.claw:
                continue
            if self.claw.check_claw_collision(obj, new_x, new_y):
                return
        
        self.claw.move(dx, dy)
        
    def attract_particles(self):
        if self.claw.active:            
            for i in range(self.def_object.num_x):
                for j in range(self.def_object.num_y):
                    p = self.def_object.p[i, j]
                    dist = np.linalg.norm(self.claw.center - p)
                    if dist < self.attract_radius:
                        force = self.attract_force / (dist ** 2)
                        direction = (self.claw.center - p) / dist
                        self.def_object.ext_force[i, j] = force * direction
                        
    def on_draw(self):
        super().on_draw()
        
        if self.claw and self.claw.active:
            claw_pos = self.claw.center
            claw_screen = (claw_pos[0] * self.scale + self.offset, 
                        claw_pos[1] * self.scale + self.offset)
            
            # Enable transparency
            pyglet.gl.glEnable(pyglet.gl.GL_BLEND)
            pyglet.gl.glBlendFunc(pyglet.gl.GL_SRC_ALPHA, pyglet.gl.GL_ONE_MINUS_SRC_ALPHA)
            
            for i in range(self.def_object.num_x):
                for j in range(self.def_object.num_y):
                    p = self.def_object.p[i, j]
                    dist = np.linalg.norm(p - claw_pos)
                    if dist < self.attract_radius:
                        p_screen = (p[0] * self.scale + self.offset, 
                                    p[1] * self.scale + self.offset)
                        
                        # RGBA color (red with 50% opacity)
                        color = (0, 0, 0, 10)
                        
                        # Draw line with custom vertex list
                        pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                            ('v2f', (*p_screen, *claw_screen)),
                            ('c4B', color * 2)  # Repeat color for both vertices
                        )

    def update(self, dt):
        if self.claw:
            self.move_claw()
            self.attract_particles()
        super().update(dt)
