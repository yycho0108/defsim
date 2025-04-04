from dataclasses import dataclass
import numpy as np

from infra.render import Config
from infra.config import zen_cli
from simulation import ClawSimulation
from objects import Line, Square, Circle, Claw, DefObject

@dataclass
class AppConfig(Config):
    window_size: tuple[int, int] = (1000, 750)
    dt: float = 0.0005
    iterations:int = 5
    self_collision: bool = True

def init_sim(cfg: AppConfig):
    sim = ClawSimulation(
        "claw machine",
        dt=cfg.dt,
        iterations=cfg.iterations,
        self_collision=cfg.self_collision,
        window_size=cfg.window_size
    )
    
    claw = Claw(center=(-0.5, 0.0), size=(0.6, 0.2), color=(0, 0, 0))
    sim.add_claw(claw)

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
    
    return sim

@zen_cli
def main(cfg: AppConfig = AppConfig()):

    sim = init_sim(cfg)
    
    # set rigid doll
    doll = DefObject(num=(1, 1), spacing=0.3, origin=(-0.5, -0.8), KS=1.0, KC=1.0)
    sim.set_def_object(doll)

    # set deformable doll
    # doll = DefObject(num=(5, 5), spacing=0.05, origin=(-0.5, -0.8), KS=0.7, KC=1.0)
    # sim.set_def_object(doll)

    """
    PBD with rigid object
    """

    def make_predictions_func(x: np.ndarray, v: np.ndarray, a: np.ndarray, G: float, dt: float) -> np.ndarray:
        """
        @param x: current positions     (shape: (num_x, num_y, 2))
        @param v: current velocities    (shape: (num_x, num_y, 2))
        @param a: current accelerations (shape: (num_x, num_y, 2))
        @param G: gravity
        @param dt: time step

        @return: tuple of predicted positions, velocities, and accelerations
        """

        p = x.copy()
        v_new = v.copy()
        a_new = a.copy()

        ############ Implement ################
        pass
        #######################################

        return p, v_new, a_new

    def apply_correction_func(x: np.ndarray, p: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """
        @param x: current positions     (shape: (num_x, num_y, 2))
        @param p: predicted positions   (shape: (num_x, num_y, 2))
        @param dt: time step

        @return: tuple of corrected positions and velocities
        """

        x_new = x.copy()
        v_new = np.zeros_like(x)

        ############ Implement ################
        pass
        #######################################

        return (x_new, v_new)


    doll.set_make_predictions_func(make_predictions_func)
    doll.set_apply_correction_func(apply_correction_func)
    
    """
    PBD with deformable object
    """

    def solve_stretching_constraint_func(p: np.ndarray, edges:list, KS: float) -> np.ndarray:
        """
        @param p: predicted positions   (shape: (num_x, num_y, 2))
        @param edges: edges of the deformable object. rest_len is the rest length of the edge
        @param KS: stretching stiffness

        @return : corrected positions
        """

        p_new = p.copy()

        for (i1, j1), (i2, j2), rest_len in edges:
            ############ Implement ################
            pass
            #######################################

        return p_new

    def solve_self_collision_constraints_func(p: np.ndarray, collision_radius: float, KC: float) -> np.ndarray:
        """
        @param p: predicted positions   (shape: (num_x, num_y, 2))
        @param collision_radius: collision radius
        @param KC: collision stiffness

        @return: corrected positions
        """

        p_new = p.copy()

        ############ Implement ################
        pass
        #######################################
        
        return p_new
    
    doll.set_solve_stretching_constraint_func(solve_stretching_constraint_func)
    doll.set_solve_self_collision_constraints_func(solve_self_collision_constraints_func)
    
    sim.run()

if __name__ == "__main__":
    main()
