# cloth.py

import numpy as np
from math import sqrt, acos
import pyglet

class Cloth:
    def __init__(self, num, spacing, origin, line_color=(0, 0, 1), point_color=(1, 0, 0), KS=0.6, KC=0.4, DAMPING=0.9):
        self.num_x = num[0]
        self.num_y = num[1]
        self.spacing = spacing
        self.origin = np.array(origin, dtype=np.float32)
        self.line_color = line_color
        self.point_color = point_color
        self.KS = KS   # Stretch
        self.KC = KC   # Collision
        self.DAMPING = DAMPING

        self.x = np.zeros((self.num_x, self.num_y, 2), dtype=np.float32)    # positions
        self.p = np.zeros((self.num_x, self.num_y, 2), dtype=np.float32)    # predicted positions
        self.v = np.zeros((self.num_x, self.num_y, 2), dtype=np.float32)    # velocities
        self.dv = np.zeros((self.num_x, self.num_y, 2), dtype=np.float32)   # delta velocities
        self.reset_pos()
                
        # initialize stretch constraints
        self.edges = []
        self.build_edges()
        
        # initialize self collision constraints
        self.triangles = []
        self.build_triangles()
        
        self._point_positions = np.zeros((self.num_x * self.num_y, 2), dtype=np.float32)
        self._line_positions = np.zeros((len(self.edges)*2, 2), dtype=np.float32)

    """
    draws the cloth
    """
    
    def draw(self, scene, scale, offset):
        p255 = tuple(int(c * 255) for c in self.point_color)
        l255 = tuple(int(c * 255) for c in self.line_color)
        
        if not hasattr(self, '_point_list'):
            num_points = self.num_x * self.num_y
            self._update_point_buffer(scale, offset)
            self._update_line_buffer(scale, offset)
            
            self._point_list = scene.add(
                num_points,
                pyglet.gl.GL_POINTS,
                None,
                ('v2f/stream', self._point_positions.flatten().tolist()),
                ('c3B/static', p255 * num_points)
            )
            self._line_list = scene.add(
                len(self.edges)*2,
                pyglet.gl.GL_LINES,
                None,
                ('v2f/stream', self._line_positions.flatten().tolist()),
                ('c3B/static', l255 * len(self.edges)*2)
            )
        else:
            self._update_point_buffer(scale, offset)
            self._update_line_buffer(scale, offset)
            self._point_list.vertices[:] = self._point_positions.flatten().tolist()
            self._line_list.vertices[:] = self._line_positions.flatten().tolist()
            
        pyglet.gl.glEnable(pyglet.gl.GL_PROGRAM_POINT_SIZE)
        pyglet.gl.glPointSize(10)
        pyglet.gl.glLineWidth(2)
    
    def _update_point_buffer(self, scale, offset):
        self._point_positions[:] = (self.x.reshape(-1, 2) * scale + offset)

    def _update_line_buffer(self, scale, offset):
        line_indices = [(i1, j1, i2, j2) for (i1, j1), (i2, j2), _ in self.edges]
        i, j, k, l = np.array(line_indices).T
        lines = np.hstack([self.x[i,j], self.x[k,l]]) * scale + offset
        self._line_positions[:] = lines.reshape(-1, 2)
    
    def reset_pos(self):
        for i in range(self.num_x):
            for j in range(self.num_y):
                self.x[i, j] = self.origin + np.array([i * self.spacing, j * self.spacing]) - np.array([self.num_x * self.spacing / 2, self.num_y * self.spacing / 2])
                self.p[i, j] = self.x[i, j]
                self.v[i, j] = np.zeros(2, dtype=np.float32)

    def external_forces(self, G, wind, DT):
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
                    
    def build_triangles(self):
        for i in range(self.num_x - 1):
            for j in range(self.num_y - 1):
                self.triangles.append(((i, j), (i+1, j), (i, j+1)))
                self.triangles.append(((i+1, j), (i, j+1), (i+1, j+1)))

    def solve_stretching_constraint(self, iterations):
        KS = self.KS ** iterations
        
        for (i1, j1), (i2, j2), rest_len in self.edges:
            p1 = self.p[i1, j1]
            p2 = self.p[i2, j2]
            
            delta = p2 - p1
            curr_len = np.linalg.norm(delta)
            n = delta / curr_len
            
            lagrange = (curr_len - rest_len) / 2
            
            self.p[i1, j1] += KS * lagrange * n
            self.p[i2, j2] -= KS * lagrange * n

    def solve_collision_constraints(self, obj):
        for i in range(self.num_x):
            for j in range(self.num_y):
                p = self.p[i, j]
                x = self.x[i, j]
                
                corr = obj.solve_collision_constraint(p, x)
                self.p[i, j] += self.KC * corr
                
    def point_in_triangle(self, p, v1, v2, v3):
        # Compute vectors
        v0 = v3 - v1
        v1_vec = v2 - v1
        v2_vec = p - v1
        
        # Compute dot products
        dot00 = np.dot(v0, v0)
        dot01 = np.dot(v0, v1_vec)
        dot02 = np.dot(v0, v2_vec)
        dot11 = np.dot(v1_vec, v1_vec)
        dot12 = np.dot(v1_vec, v2_vec)
        
        # Compute barycentric coordinates
        inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01)
        u = (dot11 * dot02 - dot01 * dot12) * inv_denom
        v = (dot00 * dot12 - dot01 * dot02) * inv_denom
        
        # Check if point is in triangle
        return (u >= -1e-6) and (v >= -1e-6) and (u + v <= 1 + 1e-6)
    
    def resolve_triangle_collision(self, p, A, B, C, i, j, A_idx, B_idx, C_idx):
        # Calculate triangle normal
        AB = B - A
        AC = C - A
        normal = np.cross(np.append(AB, 0), np.append(AC, 0))
        normal = normal[:2]  # Take only x,y components
        normal_len = np.linalg.norm(normal)
        
        if normal_len < 1e-10:
            return
            
        normal = normal / normal_len
        
        # Calculate penetration depth along normal
        penetration = np.dot(normal, A - p)
        
        if penetration <= 0:
            return  # No penetration
                
        # Apply correction with damping
        correction = (self.KC * penetration * normal) 
        point_correction = correction * (3.0 / 4.0)
        vertex_correction = -correction * (1.0 / 4.0)
        
        self.p[i, j] += point_correction
        self.p[A_idx] += vertex_correction
        self.p[B_idx] += vertex_correction
        self.p[C_idx] += vertex_correction
        
    # def solve_self_collision_constraints(self, iterations):
    #     # Create spatial hash grid for accelerated collision detection
    #     cell_size = 2 * self.spacing
    #     grid = {}
        
    #     # Insert triangles into spatial grid
    #     for idx, tri in enumerate(self.triangles):
    #         A_idx, B_idx, C_idx = tri
    #         A = self.p[A_idx]
    #         B = self.p[B_idx]
    #         C = self.p[C_idx]
            
    #         # Calculate bounding box of triangle
    #         min_x = min(A[0], B[0], C[0])
    #         min_y = min(A[1], B[1], C[1])
    #         max_x = max(A[0], B[0], C[0])
    #         max_y = max(A[1], B[1], C[1])
            
    #         # Add triangle to all cells it overlaps
    #         for x in range(int(min_x / cell_size), int(max_x / cell_size) + 1):
    #             for y in range(int(min_y / cell_size), int(max_y / cell_size) + 1):
    #                 key = (x, y)
    #                 if key not in grid:
    #                     grid[key] = []
    #                 grid[key].append(idx)
        
    #     # Check point-triangle collisions using spatial grid
    #     for i in range(self.num_x):
    #         for j in range(self.num_y):
    #             p = self.p[i, j]
                
    #             # Get cell key for this point
    #             cell_x = int(p[0] / cell_size)
    #             cell_y = int(p[1] / cell_size)
                
    #             # Check only triangles in this cell and neighboring cells
    #             for dx in [-1, 0, 1]:
    #                 for dy in [-1, 0, 1]:
    #                     key = (cell_x + dx, cell_y + dy)
    #                     if key not in grid:
    #                         continue
                        
    #                     for tri_idx in grid[key]:
    #                         tri = self.triangles[tri_idx]
    #                         if (i, j) in tri:
    #                             continue
                            
    #                         A_idx, B_idx, C_idx = tri
    #                         A = self.p[A_idx]
    #                         B = self.p[B_idx]
    #                         C = self.p[C_idx]
                            
    #                         # Continue with improved collision detection and response
    #                         if self.point_in_triangle(p, A, B, C):
    #                             self.resolve_triangle_collision(p, A, B, C, i, j, A_idx, B_idx, C_idx)
                
    def solve_self_collision_constraints(self, iterations):
        for i in range(self.num_x):
            for j in range(self.num_y):
                p = self.p[i, j]
                
                for tri in self.triangles:
                    if (i, j) in tri:
                        continue
                    
                    A_idx, B_idx, C_idx = tri
                    A = self.p[A_idx]
                    B = self.p[B_idx]
                    C = self.p[C_idx]
                    
                    centroid = (A + B + C) / 3
                    dist = np.linalg.norm(p - centroid)
                    if dist > 2 * self.spacing: # skip if too far
                        continue
                    
                    if self.point_in_triangle(p, A, B, C):
                        edges = [(A, B, C), (B, C, A), (C, A, B)]
                        max_penetration = 0
                        best_normal = None
                        
                        for P, Q, R in edges:
                            edge = Q - P
                            normal = np.array([-edge[1], edge[0]])
                            normal /= np.linalg.norm(normal)
                            
                            if np.dot(normal, R - P) > 1e-6:
                                normal = -normal
                                
                            penetration = -np.dot(normal, p - P)
                            if penetration > max_penetration:
                                max_penetration = penetration
                                best_normal = normal
                                
                        if max_penetration > 0:
                            breakpoint()
                            delta = max_penetration * best_normal
                            self.p[i, j] += self.KC * delta
                            self.p[A_idx] -= self.KC * delta / 3
                            self.p[B_idx] -= self.KC * delta / 3
                            self.p[C_idx] -= self.KC * delta / 3