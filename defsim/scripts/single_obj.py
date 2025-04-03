from dataclasses import dataclass

from simulation import Simulation
from objects import Line, DefObject
from infra.render import Config
from infra.config import zen_cli

@dataclass
class AppConfig(Config):
    window_size: tuple[int, int] = (1000, 750)
    dt: float = 0.0005
    iterations:int = 5
    self_collision: bool = True

@zen_cli
def main(cfg: AppConfig = AppConfig()):
    sim = Simulation(
        "single object",
        dt=cfg.dt,
        iterations=cfg.iterations,
        window_size=cfg.window_size,
        self_collision=cfg.self_collision
    )
    
    # set def_obj
    def_obj = DefObject(num=(5, 5), spacing=0.05, origin=(0.0, 0.0), KS=1.0, KC=1.0)
    sim.set_def_object(def_obj)

    # add line
    line = Line(center=(0.0, -0.5), normal=(0,1), color=(0.3, 0.3, 0.3))
    sim.add_object(line)
    
    # add square
    # square = Square(center=(0.0, -0.5), size=(0.5, 0.1), color=(0.3, 0.3, 0.3))
    # sim.add_object(square)
    
    # add circle
    # circle = Circle(center=(0.0, -0.5), radius=0.1, color=(0.3, 0.3, 0.3))
    # sim.add_object(circle)
    
    sim.set_wind((1.0, 0.0))

    sim.run()

if __name__ == "__main__":
    main()
