from pyglet.window import key
from .Simulation import Simulation

class ClawSimulation(Simulation):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        self.move_speed = 0.01
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

    def update_claw(self):
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

    def update(self, dt):
        super().update(dt)
        if self.claw:
            self.update_claw()
