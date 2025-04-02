# DefObject.py

import numpy as np
from math import sqrt, acos
import pyglet

class DefObject:
    def __init__(self, num, spacing, origin, line_color=(0, 0, 1), point_color=(1, 0, 0), KS=1.0, KC=1.0, DAMPING=0.9):
        self.num_x = num[0]
        self.num_y = num[1]
        self.spacing = spacing
        self.collision_radius = spacing * 0.5   # FIXME: maybe larger
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
        
        # draw buffers
        self._line_positions = np.zeros((len(self.edges)*2, 2), dtype=np.float32)
        self._point_positions = np.zeros((self.num_x * self.num_y, 2), dtype=np.float32)

    """
    draws the object
    """
    
    def draw(self, scene, scale, offset):
        p255 = tuple(int(c * 255) for c in self.point_color)
        l255 = tuple(int(c * 255) for c in self.line_color)
        
        if not hasattr(self, '_point_list'):
            num_points = self.num_x * self.num_y
            self._update_line_buffer(scale, offset)
            self._update_point_buffer(scale, offset)
            
            self._line_list = scene.add(
                len(self.edges)*2,
                pyglet.gl.GL_LINES,
                None,
                ('v2f/stream', self._line_positions.flatten().tolist()),
                ('c3B/static', l255 * len(self.edges)*2)
            )
            self._point_list = scene.add(
                num_points,
                pyglet.gl.GL_POINTS,
                None,
                ('v2f/stream', self._point_positions.flatten().tolist()),
                ('c3B/static', p255 * num_points)
            )
        else:
            self._update_line_buffer(scale, offset)
            self._update_point_buffer(scale, offset)
            self._line_list.vertices[:] = self._line_positions.flatten().tolist()
            self._point_list.vertices[:] = self._point_positions.flatten().tolist()
            
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
        inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01 + 1e-6)
        u = (dot11 * dot02 - dot01 * dot12) * inv_denom
        v = (dot00 * dot12 - dot01 * dot02) * inv_denom
        
        # Check if point is in triangle
        return (u >= -1e-6) and (v >= -1e-6) and (u + v <= 1 + 1e-6)
                
    def resolve_self_collsion(self, p, x, a, b, c):
        p_is_inside = self.point_in_triangle(p, a, b, c)
        x_is_inside = self.point_in_triangle(x, a, b, c)
        
        if not p_is_inside:
            return False, None, None
        
        elif x_is_inside:
            # find the closest point on the triangle
            penetration_depth = float('inf')
            closest_normal = None
            
            edges = [(a, b), (b, c), (c, a)]
            for e1, e2 in edges:
                edge_vec = e2 - e1
                edge_len = np.linalg.norm(edge_vec)
                if edge_len < 1e-6:
                    continue
                
                dist = np.linalg.norm(np.cross(edge_vec, e1 - p)) / edge_len
                
                if dist < penetration_depth:
                    penetration_depth = dist
                    normal = (p - e1) - np.dot((p - e1), edge_vec) * edge_vec / edge_len
                    if np.linalg.norm(normal) > 1e-6:
                        closest_normal = normal / np.linalg.norm(normal)
                        
            if closest_normal is None:
                return False, None, None
                    
            return True, closest_normal, penetration_depth
            
        else:
            # find penetrated edge
            edges = [(a, b), (b, c), (c, a)]
            for e1, e2 in edges:
                xp = p - x
                ab = e2 - e1
                ax = x - e1
            
                det = -xp[0] * ab[1] + xp[1] * ab[0]
                if abs(det) < 1e-6:
                    continue
            
                t = (ax[0] * ab[1] - ax[1] * ab[0]) / det
                k = (-xp[0] * ax[1] + xp[1] * ax[0]) / det
                
                # penetration
                if 0 < t < 1 and 0 < k < 1:
                    normal = np.array([-ab[1], ab[0]])
                    if np.dot(normal, xp) > 0:
                        normal = -normal
                    normal /= np.linalg.norm(normal)
                    penetration_depth = -np.dot(normal, xp)
                    
                    return True, normal, penetration_depth
            
            return False, None, None
    
    def solve_self_collision_constraints(self, iterations):
        for i in range(self.num_x):
            for j in range(self.num_y):
                # current point
                p = self.p[i, j]
                x = self.x[i, j]
                
                for tri in self.triangles:
                    idx1, idx2, idx3 = tri
                    
                    # TODO: filter out some traingles
                    if (i, j) in tri:
                        continue
                    
                    v1, v2, v3 = [self.p[i1, j1] for i1, j1 in tri]
                    
                    # FIXME: do we need to check this??
                    # v1o, v2o, v3o = [self.x[i1, j1] for i1, j1 in tri]
                    
                    min_x = min(v1[0], v2[0], v3[0])
                    max_x = max(v1[0], v2[0], v3[0])
                    min_y = min(v1[1], v2[1], v3[1])
                    max_y = max(v1[1], v2[1], v3[1])
                    
                    margin = self.spacing * 0.5
                    if (p[0] < min_x - margin or p[0] > max_x + margin or 
                        p[1] < min_y - margin or p[1] > max_y + margin):
                        continue
                    
                    penetrate, normal, depth = self.resolve_self_collsion(p, x, v1, v2, v3)
                    
                    if penetrate and depth > 0:
                        # apply correction
                        delta = normal * depth
                        self.p[i, j] += self.KC * delta * 3 / 4
                        self.p[idx1] -= self.KC * delta / 4
                        self.p[idx2] -= self.KC * delta / 4
                        self.p[idx3] -= self.KC * delta / 4
                        # breakpoint()