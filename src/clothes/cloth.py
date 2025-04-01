# src/clothes/cloth.py

import numpy as np

class Cloth:
    """
    2D cloth simulation using PBD constraints (stretch, collision, wind, etc.)
    """

    def __init__(
        self,
        nx=20,
        ny=10,
        size_x=0.8,
        size_y=0.5,
        gravity=-9.8,
        damping=0.99,
        ks=0.6,
        kb=0.1,
        dt=0.01,
        wind=(0.0, 0.0)
    ):
        self.nx = nx
        self.ny = ny
        self.size_x = size_x
        self.size_y = size_y

        self.gravity = gravity
        self.damping = damping
        self.ks = ks   # stretch stiffness
        self.kb = kb   # bending stiffness (not fully used)
        self.dt = dt

        # wind force (constant for each particle)
        self.wind = np.array(wind, dtype=np.float32)

        # positions, velocities, and fixed flags
        self.positions = np.zeros((nx * ny, 2), dtype=np.float32)
        self.velocities = np.zeros((nx * ny, 2), dtype=np.float32)
        self.fixed = np.zeros((nx * ny,), dtype=np.int32)

        self.init_positions()
        self.edges = self.make_edges()

        # rest_lengths for each edge
        self.rest_lengths = None

        # externally assigned collision objects
        self.collision_objects = []

    def init_positions(self):
        # distribute points in a grid
        dx = self.size_x / (self.nx - 1) if self.nx > 1 else 0.0
        dy = self.size_y / (self.ny - 1) if self.ny > 1 else 0.0

        idx = 0
        for i in range(self.nx):
            for j in range(self.ny):
                x = i * dx - self.size_x * 0.5
                y = j * dy
                self.positions[idx, 0] = x
                self.positions[idx, 1] = y
                self.velocities[idx, :] = 0.0
                idx += 1

        # example: fix top corners
        self.fix_point(0)
        self.fix_point(self.ny - 1)

    def index_of(self, i, j):
        return i * self.ny + j

    def make_edges(self):
        edge_list = []
        for i in range(self.nx):
            for j in range(self.ny):
                if i + 1 < self.nx:
                    edge_list.append((self.index_of(i, j), self.index_of(i + 1, j)))
                if j + 1 < self.ny:
                    edge_list.append((self.index_of(i, j), self.index_of(i, j + 1)))
        return edge_list

    def fix_point(self, idx):
        self.fixed[idx] = 1

    def set_wind(self, wind):
        """ Allows runtime modification of wind force. """
        self.wind = np.array(wind, dtype=np.float32)

    def step(self, substeps=5):
        # 1) external forces and damping
        #    gravity in y-direction + wind in (x,y)
        for i in range(self.nx * self.ny):
            if self.fixed[i] == 0:
                self.velocities[i, 1] += self.gravity * self.dt  # gravity
                self.velocities[i] += self.wind * self.dt        # wind
            self.velocities[i] *= self.damping

        # 2) predict positions
        p_pred = self.positions + self.velocities * self.dt

        # 3) constraints repeated
        for _ in range(substeps):
            p_pred = self.solve_stretch_constraints(p_pred)
            p_pred = self.solve_collision(p_pred)

        # 4) update velocities and positions
        new_vel = (p_pred - self.positions) / self.dt
        self.velocities = new_vel
        self.positions = p_pred

        # restore fixed points
        for i in range(self.nx * self.ny):
            if self.fixed[i] == 1:
                self.velocities[i, :] = 0.0

    def solve_stretch_constraints(self, p_pred):
        if self.rest_lengths is None:
            self.rest_lengths = []
            for e in self.edges:
                i1, i2 = e
                rest_len = np.linalg.norm(self.positions[i1] - self.positions[i2])
                self.rest_lengths.append(rest_len)

        out = p_pred.copy()
        for edge_idx, (i1, i2) in enumerate(self.edges):
            if self.fixed[i1] == 1 and self.fixed[i2] == 1:
                continue

            rest_len = self.rest_lengths[edge_idx]
            p1 = out[i1]
            p2 = out[i2]
            dir_vec = p2 - p1
            dist = np.linalg.norm(dir_vec)
            if dist < 1e-8:
                continue

            n = dir_vec / dist
            diff = dist - rest_len
            correction = 0.5 * self.ks * diff * n

            if self.fixed[i1] == 0:
                out[i1] += correction
            if self.fixed[i2] == 0:
                out[i2] -= correction

        return out

    def solve_collision(self, p_pred):
        out = p_pred.copy()
        for i in range(self.nx * self.ny):
            if self.fixed[i] == 1:
                continue
            # check collision with all objects
            for obj in self.collision_objects:
                corr = obj.solve_collision_constraint(out[i], self.positions[i])
                out[i] += corr
        return out

    def draw(self, batch, group=None):
        """
        Draw lines (edges) using pyglet graphics (Batch).
        """
        lines = []
        scale = 300.0
        offset_x = 400.0
        offset_y = 300.0

        for e in self.edges:
            i1, i2 = e
            x1, y1 = self.positions[i1]
            x2, y2 = self.positions[i2]
            lines.append((x1*scale + offset_x, y1*scale + offset_y,
                          x2*scale + offset_x, y2*scale + offset_y))

        vertex_count = len(lines)*2
        vertex_data = []
        for l in lines:
            vertex_data.extend([l[0], l[1], l[2], l[3]])

        # GL_LINES = 1
        batch.add(
            vertex_count,
            1,
            group,
            ('v2f', vertex_data)
        )
