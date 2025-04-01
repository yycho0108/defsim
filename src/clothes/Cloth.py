# cloth.py

import numpy as np
from math import sqrt, acos
import pyglet
from pyglet import shapes

class Cloth:
    def __init__(self, num, spacing, origin, color=(1,1,1), KS=0.6, KC=0.4, DAMPING=0.9):
        self.num_x = num[0]
        self.num_y = num[1]
        self.spacing = spacing
        self.origin = np.array(origin, dtype=np.float32)
        self.color = color
        self.KS = KS   # Stretch
        self.KC = KC   # Collision
        self.DAMPING = DAMPING

        self.x = np.zeros((self.num_x, self.num_y, 2), dtype=np.float32)    # positions
        self.p = np.zeros((self.num_x, self.num_y, 2), dtype=np.float32)    # predicted positions
        self.v = np.zeros((self.num_x, self.num_y, 2), dtype=np.float32)    # velocities
        self.reset_pos()
                
        self.stretch_constraints = []
        self.build_stretch_constraints()

    """
    draws the cloth
    """
    def draw(self, scene, scale):
        r255 = tuple(int(c * 255) for c in self.color)
        
        if not hasattr(self, 'circle_shapes'):
            self.circle_shapes = []
            self.line_shapes = []
            
            for i in range(self.num_x):
                for j in range(self.num_y):
                    x, y = self.x[i, j] * scale
                    circle = shapes.Circle(x, y, radius=3, color=r255, batch=scene)
                    self.circle_shapes.append(circle)
            
            for (i1, j1), (i2, j2), _ in self.stretch_constraints:
                x1, y1 = self.x[i1, j1] * scale
                x2, y2 = self.x[i2, j2] * scale
                line = shapes.Line(x1, y1, x2, y2, color=r255, batch=scene)
                self.line_shapes.append(line)
        
        else:
            idx = 0
            for i in range(self.num_x):
                for j in range(self.num_y):
                    x, y = self.x[i, j] * scale
                    self.circle_shapes[idx].x = x
                    self.circle_shapes[idx].y = y
                    idx += 1
            
            for line_idx, ((i1, j1), (i2, j2), _) in enumerate(self.stretch_constraints):
                x1, y1 = self.x[i1, j1] * scale
                x2, y2 = self.x[i2, j2] * scale
                self.line_shapes[line_idx].x = x1
                self.line_shapes[line_idx].y = y1
                self.line_shapes[line_idx].x2 = x2
                self.line_shapes[line_idx].y2 = y2
    
    def reset_pos(self):
        for i in range(self.num_x):
            for j in range(self.num_y):
                self.x[i, j] = self.origin + np.array([i * self.spacing, -j * self.spacing])
                self.p[i, j] = self.x[i, j]
                self.v[i, j] = np.zeros(2, dtype=np.float32)

    def external_forces(self, G, wind, DT):
        self.dv = np.zeros((self.num_x, self.num_y, 2), dtype=np.float32)
        self.dv[:, :, 1] += G * DT
        self.dv += wind * DT
        self.v += self.DAMPING * self.dv

    def make_predictions(self, DT):
        self.p = self.x + DT * self.v

    def apply_correction(self, DT):
        for i in range(self.num_x):
            for j in range(self.num_y):
                self.v[i, j] = (self.x[i, j] - self.p[i, j]) / DT
                self.x[i, j] = self.p[i, j]
                
    def build_stretch_constraints(self):
        for i in range(self.num_x):
            for j in range(self.num_y):
                if i < self.num_x - 1:
                    p1 = self.p[i, j]
                    p2 = self.p[i+1, j]
                    rest_len = np.linalg.norm(p2 - p1)
                    self.stretch_constraints.append(((i, j), (i+1, j), rest_len))
                if j < self.num_y - 1:
                    p1 = self.p[i, j]
                    p2 = self.p[i, j+1]
                    rest_len = np.linalg.norm(p2 - p1)
                    self.stretch_constraints.append(((i, j), (i, j+1), rest_len))
                # Diagonal constraints for shape preservation
                if i < self.num_x - 1 and j < self.num_y - 1:
                    p1 = self.p[i, j]
                    p2 = self.p[i+1, j+1]
                    rest_len = np.linalg.norm(p2 - p1)
                    self.stretch_constraints.append(((i, j), (i+1, j+1), rest_len))
                if i < self.num_x - 1 and j > 0:
                    p1 = self.p[i, j]
                    p2 = self.p[i+1, j-1]
                    rest_len = np.linalg.norm(p2 - p1)
                    self.stretch_constraints.append(((i, j), (i+1, j-1), rest_len))

    def solve_stretching_constraint(self, iterations):
        KS = self.KS ** iterations
        
        for (i1, j1), (i2, j2), rest_len in self.stretch_constraints:
            p1 = self.p[i1, j1]
            p2 = self.p[i2, j2]
            
            delta = p2 - p1
            l = np.linalg.norm(delta)
            if l == 0:
                continue
            n = (l - rest_len) / l
            
            lagrange = (l - rest_len) / 2
            
            self.p[i1] -= KS * lagrange * n
            self.p[i2] += KS * lagrange * n

    def solve_collision_constraints(self, obj):
        for i in range(self.num_x):
            for j in range(self.num_y):
                p = self.p[i, j]
                x = self.x[i, j]
                
                corr = obj.solve_collision_constraint(p, x)
                print(corr)
                self.p[i] += self.KC * corr