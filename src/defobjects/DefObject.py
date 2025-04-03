# src/defobjects/DefObject.py

import numpy as np
import pyglet

class DefObject:
    def __init__(self, num, spacing, origin, line_color=(0, 0, 0), point_color=(0.5, 0.5, 0.5), KS=1.0, KC=1.0):
        self.num_x = num[0]
        self.num_y = num[1]
        self.spacing = spacing
        self.collision_radius = spacing * 0.5
        self.origin = np.array(origin, dtype=np.float32)
        self.line_color = line_color
        self.point_color = point_color
        self.KS = KS   # Stretch
        self.KC = KC   # Collision

        self.x = np.zeros((self.num_x, self.num_y, 2), dtype=np.float32)            # positions
        self.p = np.zeros((self.num_x, self.num_y, 2), dtype=np.float32)            # predicted positions
        self.v = np.zeros((self.num_x, self.num_y, 2), dtype=np.float32)            # velocities
        self.dv = np.zeros((self.num_x, self.num_y, 2), dtype=np.float32)           # delta velocities
        self.reset_pos()
                
        # initialize stretch constraints
        self.edges = []
        self.build_edges()
        
        # draw buffers
        self._line_positions = np.zeros((len(self.edges)*2, 2), dtype=np.float32)
        self._point_positions = np.zeros((self.num_x * self.num_y, 2), dtype=np.float32)

    """
    draws the object
    """
    
    def draw(self, scene, scale, offset):
        p255 = tuple(int(c * 255) for c in self.point_color)
        l255 = tuple(int(c * 255) for c in self.line_color)
        
        # Update line buffer regardless
        self._update_line_buffer(scale, offset)
        
        if not hasattr(self, '_line_list'):
            # Setup lines
            self._line_list = scene.add(
                len(self.edges)*2,
                pyglet.gl.GL_LINES,
                None,
                ('v2f/stream', self._line_positions.flatten().tolist()),
                ('c3B/static', l255 * len(self.edges)*2)
            )
            
            # Create circles using pyglet.shapes
            self._circles = []
            for i in range(self.num_x):
                for j in range(self.num_y):
                    pos = self.p[i, j] * scale + offset
                    circle = pyglet.shapes.Circle(
                        pos[0], pos[1],
                        self.collision_radius * scale,
                        segments=16,
                        color=p255,
                        batch=scene
                    )
                    self._circles.append((circle, (i, j)))
        else:
            # Update lines
            self._line_list.vertices[:] = self._line_positions.flatten().tolist()
            
            # Update circle positions
            for circle, (i, j) in self._circles:
                pos = self.p[i, j] * scale + offset
                circle.x = pos[0]
                circle.y = pos[1]
                circle.radius = self.collision_radius * scale
        
        # Set rendering properties
        pyglet.gl.glLineWidth(5)

    def _update_line_buffer(self, scale, offset):
        line_indices = [(i1, j1, i2, j2) for (i1, j1), (i2, j2), _ in self.edges]
        i, j, k, l = np.array(line_indices).T
        lines = np.hstack([self.x[i,j], self.x[k,l]]) * scale + offset
        self._line_positions[:] = lines.reshape(-1, 2)

    def reset_pos(self):
        offset_x = (self.num_x - 1) * self.spacing / 2
        offset_y = (self.num_y - 1) * self.spacing / 2
        for i in range(self.num_x):
            for j in range(self.num_y):
                self.x[i, j] = self.origin + np.array([i * self.spacing, j * self.spacing]) - np.array([offset_x, offset_y])
                self.p[i, j] = self.x[i, j]
                self.v[i, j] = np.zeros(2, dtype=np.float32)


    def external_forces(self, G, WIND, DT):
        # self.dv.fill(0)
        self.dv[:, :, 1] += G * DT
        self.dv += WIND * DT
        self.v += self.dv

    def make_predictions(self, DT):
        self.p = self.x + DT * self.v

    def apply_correction(self, DT):
        for i in range(self.num_x):
            for j in range(self.num_y):
                self.v[i, j] = (self.p[i, j] - self.x[i, j]) / DT
                self.x[i, j] = self.p[i, j]
                
    def build_edges(self):
        for i in range(self.num_x):
            for j in range(self.num_y):
                if i < self.num_x - 1:
                    p1 = self.x[i, j]
                    p2 = self.x[i+1, j]
                    rest_len = np.linalg.norm(p2 - p1)
                    self.edges.append(((i, j), (i+1, j), rest_len))
                if j < self.num_y - 1:
                    p1 = self.x[i, j]
                    p2 = self.x[i, j+1]
                    rest_len = np.linalg.norm(p2 - p1)
                    self.edges.append(((i, j), (i, j+1), rest_len))
                # Diagonal constraints for shape preservation
                if i < self.num_x - 1 and j < self.num_y - 1:
                    p1 = self.x[i, j]
                    p2 = self.x[i+1, j+1]
                    rest_len = np.linalg.norm(p2 - p1)
                    self.edges.append(((i, j), (i+1, j+1), rest_len))
                if i < self.num_x - 1 and j > 0:
                    p1 = self.x[i, j]
                    p2 = self.x[i+1, j-1]
                    rest_len = np.linalg.norm(p2 - p1)
                    self.edges.append(((i, j), (i+1, j-1), rest_len))

    def solve_stretching_constraint(self, iterations):
        KS = self.KS ** iterations
        
        for (i1, j1), (i2, j2), rest_len in self.edges:
            p1 = self.p[i1, j1]
            p2 = self.p[i2, j2]
            
            delta = p2 - p1
            curr_len = np.linalg.norm(delta)
            if curr_len < 1e-6:
                continue
            n = delta / curr_len
            
            lagrange = (curr_len - rest_len) / 2
            
            self.p[i1, j1] += KS * lagrange * n
            self.p[i2, j2] -= KS * lagrange * n

    def solve_collision_constraints(self, obj):
        for i in range(self.num_x):
            for j in range(self.num_y):
                p = self.p[i, j]                
                corr = obj.solve_collision_constraint(p, self.collision_radius)
                self.p[i, j] += self.KC * corr

    def solve_self_collision_constraints(self, iterations):
        for i in range(self.num_x):
            for j in range(self.num_y):
                # current point
                p = self.p[i, j]
                
                # Check against all other points
                for i2 in range(self.num_x):
                    for j2 in range(self.num_y):
                        # Skip self
                        if i == i2 and j == j2:
                            continue
                        
                        # Skip immediate neighbors (connected by constraints)
                        if (abs(i - i2) <= 1 and abs(j - j2) <= 1):
                            continue
                        
                        p2 = self.p[i2, j2]
                        
                        # Get vector between points
                        delta = p - p2
                        dist = np.linalg.norm(delta)
                        
                        # If distance is less than twice the collision radius (overlapping)
                        min_dist = self.collision_radius * 2
                        if dist < min_dist and dist > 1e-6:
                            # Direction from other point to this point
                            dir_vec = delta / dist
                            
                            # Calculate correction to push particles apart to minimum distance
                            correction = (min_dist - dist) * dir_vec * 0.5
                            
                            # Apply correction to both particles
                            self.p[i, j] += self.KC * correction
                            self.p[i2, j2] -= self.KC * correction