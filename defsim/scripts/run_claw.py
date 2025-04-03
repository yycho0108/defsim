from dataclasses import dataclass

from infra.render import Config
from infra.config import zen_cli
from simulation import ClawSimulation
from objects import Line, Square, Circle, Claw, DefObject

@dataclass
class AppConfig(Config):
    window_size: tuple[int, int] = (2000, 1500)
    dt: float = 0.0005
    iterations:int = 5
    self_collision: bool = False


@zen_cli
def main(cfg: AppConfig = AppConfig()):
    sim = ClawSimulation(
        "claw machine",
        dt=cfg.dt,
        iterations=cfg.iterations,
        self_collision=cfg.self_collision,
        window_size=cfg.window_size
    )
    
    claw = Claw(center=(-0.5, 0.0), size=(0.6, 0.2), color=(0, 0, 0))
    sim.add_claw(claw)
    
    # set doll
    doll = DefObject(num=(1, 1), spacing=0.1, origin=(-0.5, -0.8), KS=1.0, KC=1.0)
    # doll = DefObject(num=(5, 5), spacing=0.05, origin=(-0.5, -0.8), KS=0.5, KC=1.0)
    sim.set_def_object(doll)

    # add walls
    walls = []
    walls.append(Line(center=(0.0, -sim.world_height/2), normal=(0,1), color=(1, 0, 0)))
    walls.append(Line(center=(-sim.world_width/2, 0.0), normal=(1,0), color=(1, 0, 0)))
    walls.append(Line(center=(sim.world_width/2, 0.0), normal=(-1,0), color=(1, 0, 0)))
    for wall in walls:
        sim.add_object(wall)
    
    # add squares (basket)
    squares = []
    squares.append(Square(center=(0.1, -0.7), size=(0.2, 0.6), color=(0, 0, 1)))
    squares.append(Square(center=(0.9, -0.7), size=(0.2, 0.6), color=(0, 0, 1)))
    for square in squares:
        sim.add_object(square)
        
    # add circles (gate)
    circles = []
    circles.append(Circle(center=(0.22, -0.4), radius=0.22, color=(0, 0, 1)))
    circles.append(Circle(center=(0.78, -0.4), radius=0.22, color=(0, 0, 1)))
    for circle in circles:
        sim.add_object(circle)
    
    # sim.set_wind((1.0, 0.0))
    
    sim.run()

if __name__ == "__main__":
    main()
