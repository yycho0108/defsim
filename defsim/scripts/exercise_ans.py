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
    # doll = DefObject(num=(1, 1), spacing=0.3, origin=(-0.5, -0.8), KS=1.0, KC=1.0)
    # sim.set_def_object(doll)

    # set deformable doll
    doll = DefObject(num=(5, 5), spacing=0.05, origin=(-0.5, -0.8), KS=1.0, KC=1.0)
    sim.set_def_object(doll)

    """
    PBD with rigid object
    """

    def make_predictions_func(x: np.ndarray, v: np.ndarray, a: np.ndarray, G: float, dt: float) -> np.ndarray:
        """
        @param x: current positions
        @param v: current velocities
        @param a: current accelerations
        @param G: gravity
        @param dt: time step

        @return: tuple of predicted positions, velocities, and accelerations
        """

        a[:, :, 1] += G * dt
        v += a
        p = x + dt * v

        return p, v, a

    def apply_correction_func(x: np.ndarray, p: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """
        @param x: current positions
        @param p: predicted positions
        @param dt: time step

        @return: tuple of corrected positions and velocities
        """

        v = (p - x) / dt
        x = p

        return x, v
    
    doll.set_make_predictions_func(make_predictions_func)
    doll.set_apply_correction_func(apply_correction_func)

    
    """
    PBD with deformable object
    """

    def solve_stretching_constraint_func(p: np.ndarray, edges:list, KS: float) -> np.ndarray:
        """
        @param p: predicted positions
        @param edges: edges of the deformable object
        @param KS: stretching stiffness

        @return : corrected positions
        """

        p_new = p.copy()
        for (i1, j1), (i2, j2), rest_len in edges:
            p1 = p_new[i1, j1]
            p2 = p_new[i2, j2]
            
            delta = p2 - p1
            curr_len = np.linalg.norm(delta)
            if curr_len < 1e-6:
                continue
            n = delta / curr_len
            
            lagrange = (curr_len - rest_len) / 2
            
            p_new[i1, j1] += KS * lagrange * n
            p_new[i2, j2] -= KS * lagrange * n

        return p_new

    def solve_self_collision_constraints_func(p: np.ndarray, collision_radius: float, KC: float) -> np.ndarray:
        """
        @param p: predicted positions
        @param collision_radius: collision radius
        @param KC: collision stiffness

        @return: corrected positions
        """

        p_new = p.copy()

        num_x = p_new.shape[0]
        num_y = p_new.shape[1]

        for i in range(num_x):
            for j in range(num_y):
                # current point
                p_ij = p_new[i, j]
                
                # Check against all other points
                for i2 in range(num_x):
                    for j2 in range(num_y):
                        # Skip self
                        if i == i2 and j == j2:
                            continue
                        
                        # Skip immediate neighbors (connected by constraints)
                        if (abs(i - i2) <= 1 and abs(j - j2) <= 1):
                            continue
                        
                        p2 = p_new[i2, j2]
                        
                        # Get vector between points
                        delta = p_ij - p2
                        dist = np.linalg.norm(delta)
                        
                        # If distance is less than twice the collision radius (overlapping)
                        min_dist = collision_radius * 2
                        if dist < min_dist and dist > 1e-6:
                            # Direction from other point to this point
                            dir_vec = delta / dist
                            
                            # Calculate correction to push particles apart to minimum distance
                            correction = (min_dist - dist) * dir_vec * 0.5
                            
                            # Apply correction to both particles
                            p_new[i, j] += KC * correction
                            p_new[i2, j2] -= KC * correction
        
        return p_new
    

    doll.set_solve_stretching_constraint_func(solve_stretching_constraint_func)
    doll.set_solve_self_collision_constraints_func(solve_self_collision_constraints_func)
    
    sim.run()

if __name__ == "__main__":
    main()
