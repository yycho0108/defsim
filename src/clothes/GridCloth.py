# src/clothes/grid_cloth.py

import numpy as np
import pyglet
from pyglet.gl import GL_LINES

class GridCloth:
    """
    2D cloth in a grid arrangement, mimicking the old code's structure.
    """

    def __init__(
        self,
        N=(20, 20),
        cell_size=(0.5, 0.5),
        gravity=-9.8,
        dt=0.01,
        wind=(0.0, 0.0),
        color=(0.5, 0.5, 0.5),
        stiffness=0.6,
        damping=0.99,
        center=(0.0, 0.0),
    ):
        """
        N: (nx, ny)
        cell_size: (sx, sy)
        gravity: float
        dt: float
        wind: (wx, wy)
        color: (r,g,b) in [0,1]
        stiffness: stretching stiffness
        damping: velocity damping factor
        center: grid center
        """
        self.nx, self.ny = N
        self.sx, self.sy = cell_size
        self.gravity = gravity
        self.dt = dt
        self.wind = np.array(wind, dtype=np.float32)
        self.color = color
        self.stiffness = stiffness
        self.damping = damping
        self.center = np.array(center, dtype=np.float32)

        # positions & velocities
        self.positions = np.zeros((self.nx*self.ny, 2), dtype=np.float32)
        self.velocities = np.zeros((self.nx*self.ny, 2), dtype=np.float32)
        self.fixed = np.zeros((self.nx*self.ny,), dtype=np.int32)

        # edges
        self.edges = []
        self.rest_lengths = []

        # initialize scene
        self.init_scene()

    def init_scene(self):
        """
        Initialize the cloth as a 2D grid in the XY line.
        """
        dx = self.sx / (self.nx - 1) if self.nx > 1 else 0.0
        dy = self.sy / (self.ny - 1) if self.ny > 1 else 0.0

        for i in range(self.nx):
            for j in range(self.ny):
                idx = i*self.ny + j
                x = self.center[0] + (i * dx - self.sx*0.5)
                y = self.center[1] + (j * dy - self.sy*0.5)
                self.positions[idx] = [x, y]
                self.velocities[idx] = [0.0, 0.0]

        # build edges
        for i in range(self.nx):
            for j in range(self.ny):
                if i+1 < self.nx:
                    e = (self.index_of(i,j), self.index_of(i+1,j))
                    self.edges.append(e)
                if j+1 < self.ny:
                    e = (self.index_of(i,j), self.index_of(i,j+1))
                    self.edges.append(e)

        # precompute rest lengths
        for (p1, p2) in self.edges:
            rest_len = np.linalg.norm(self.positions[p1] - self.positions[p2])
            self.rest_lengths.append(rest_len)

    def index_of(self, i, j):
        return i*self.ny + j

    def fix_point(self, idx):
        self.fixed[idx] = 1

    def external_forces(self, gravity, wind, dt):
        """
        Add external forces: gravity, wind, etc.
        """
        for i in range(self.nx*self.ny):
            if self.fixed[i] == 0:
                self.velocities[i,1] += gravity * dt
                self.velocities[i] += wind * dt
            self.velocities[i] *= self.damping

    def step(self, collision_objects=[]):
        """
        One iteration of PBD:
        1) predict positions
        2) solve constraints
        3) update velocities
        """
        # predict
        p_pred = self.positions + self.velocities * self.dt

        # solve stretching
        p_pred = self.solve_stretch_constraints(p_pred)

        # solve collision
        p_pred = self.solve_collision(p_pred, collision_objects)

        # update velocity & positions
        new_vel = (p_pred - self.positions)/self.dt
        self.velocities = new_vel
        self.positions = p_pred

        # enforce fixed
        for i in range(self.nx*self.ny):
            if self.fixed[i] == 1:
                self.velocities[i] = 0.0

    def solve_stretch_constraints(self, p_pred):
        out = p_pred.copy()
        for e_idx, (p1, p2) in enumerate(self.edges):
            if self.fixed[p1] == 1 and self.fixed[p2] == 1:
                continue
            d = out[p2] - out[p1]
            dist = np.linalg.norm(d)
            if dist < 1e-8:
                continue
            rest_len = self.rest_lengths[e_idx]
            diff = dist - rest_len
            n = d / dist
            correction = 0.5 * self.stiffness * diff * n

            if self.fixed[p1] == 0:
                out[p1] += correction
            if self.fixed[p2] == 0:
                out[p2] -= correction

        return out

    def solve_collision(self, p_pred, objects):
        out = p_pred.copy()
        for i in range(self.nx*self.ny):
            if self.fixed[i] == 1:
                continue
            for obj in objects:
                corr = obj.solve_collision_constraint(out[i], self.positions[i])
                out[i] += corr
        return out

    def draw(self, scene):
        """
        Draw edges as lines
        """
        scale = 300.0
        offset_x = 400.0
        offset_y = 300.0

        # color
        r255 = tuple(int(c * 255) for c in self.color)

        # We'll do a line "by hand" or we can do a shapes.Line
        # For multiple lines, we can use the underlying batch.add with GL_LINES
        vertex_data = []
        for (p1, p2) in self.edges:
            x1, y1 = self.positions[p1]
            x2, y2 = self.positions[p2]
            vertex_data.extend([x1*scale+offset_x, y1*scale+offset_y,
                                x2*scale+offset_x, y2*scale+offset_y])

        # each edge => 2 vertices => #edges*2
        vertex_count = len(self.edges)*2

        # GL_LINES = 1
        scene.add(
            vertex_count,
            1,
            None,
            ('v2f', vertex_data),
            # color doesn't come from 'v2f' directly, so we can use shapes.Line
            # or handle color differently. We'll ignore color or
            # do a single color override with shape-based approach if needed.
        )