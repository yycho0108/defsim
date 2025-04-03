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
        self.build_edges()
        
        # draw buffers
        self._line_positions = np.zeros((len(self.edges)*2, 2), dtype=np.float32)
        self._point_positions = np.zeros((self.num_x * self.num_y, 2), dtype=np.float32)

        self.make_predictions_func = None
        self.apply_correction_func = None

        self.solve_stretching_constraint_func = None
        self.solve_self_collision_constraints_func = None

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
        if (len(self.edges) == 0):
            return
        line_indices = [(i1, j1, i2, j2) for (i1, j1), (i2, j2), _ in self.edges]
        i, j, k, l = np.array(line_indices).T
        p1 = self.x[i, j] * scale + offset
        p2 = self.x[k, l] * scale + offset
        lines = np.hstack([p1, p2])
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

    def set_make_predictions_func(self, func):
        self.make_predictions_func = func

    def make_predictions(self, DT):
        # copy to avoid modifying in place
        if self.make_predictions_func is not None:
            self.p = self.make_predictions_func(self.x.copy(), self.v.copy(), DT)

    def set_apply_correction_func(self, func):
        self.apply_correction_func = func

    def apply_correction(self, DT):
        # copy to avoid modifying in place
        if self.apply_correction_func is not None:
            self.x, self.v = self.apply_correction_func(self.x.copy(), self.p.copy(), DT)
                
    def build_edges(self):
        self.edges = []
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

    def set_solve_stretching_constraint_func(self, func):
        self.solve_stretching_constraint_func = func

    def solve_stretching_constraint(self, iterations):
        if self.solve_stretching_constraint_func is not None:
            KS = self.KS ** iterations
            self.p = self.solve_stretching_constraint_func(self.p.copy(), self.edges.copy(), KS)

    def solve_collision_constraints(self, obj):
        for i in range(self.num_x):
            for j in range(self.num_y):
                p = self.p[i, j]                
                corr = obj.solve_collision_constraint(p, self.collision_radius)
                self.p[i, j] += self.KC * corr

    def set_solve_self_collision_constraints_func(self, func):
        self.solve_self_collision_constraints_func = func

    def solve_self_collision_constraints(self, iterations):
        if self.solve_self_collision_constraints_func is not None:
            self.p = self.solve_self_collision_constraints_func(self.p.copy(), self.collision_radius, self.KC)
