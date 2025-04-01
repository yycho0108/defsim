# cloth.py

import numpy as np
from math import sqrt, acos
import pyglet
from pyglet.gl import GL_TRIANGLES

class Cloth:
    def __init__(self, V, POSITIONS, VELOCITIES, WEIGHT, indices, edges, tripairs, color=(1,1,1), KS=0.6, KC=0.4, KB=0.4, KD=0.0, KL=0.0, DAMPING=0.9):
        self.V = V
        self.POSITIONS = POSITIONS
        self.VELOCITIES = VELOCITIES
        self.indices = indices
        self.edges = edges
        self.tripairs = tripairs
        self.color = color
        self.KS = KS   # Stretch
        self.KC = KC   # Collision
        self.KB = KB   # Bending
        self.KL = KL   # Lift (?)
        self.KD = KD   # Drag
        self.DAMPING = DAMPING
        self.objs = None

        # keep track
        self.x = np.zeros((self.V, 3), dtype=np.float32)
        self.fixed = np.zeros((self.V,), dtype=np.int32)
        self.nt = np.zeros((self.V,), dtype=np.int32)
        
        for j in range(self.indices.shape[0]//3):
            vi1 = self.indices[j * 3 + 0]
            vi2 = self.indices[j * 3 + 1]
            vi3 = self.indices[j * 3 + 2]
            
            self.nt[vi1]+=1
            self.nt[vi2]+=1
            self.nt[vi3]+=1
        
        self.v = np.zeros((self.V, 3), dtype=np.float32)
        self.dv = np.zeros((self.V, 3), dtype=np.float32)
        self.np.zeros((self.V, 3), dtype=np.float32)
        self.WEIGHT = np.copy(WEIGHT)

        self.reset_pos_and_vel()

    """
    sets pos and vel to initial pos
    """
    def reset_pos_and_vel(self):
        self.x[:] = self.POSITIONS.copy()
        self.v[:] = self.VELOCITIES.copy()

    """
    draws the cloth
    """
    def draw(self, batch):
        scale = 300.0
        offset_x = 400.0
        offset_y = 300.0

        # flatten tri coords
        vertex_count = len(self.indices)
        vertex_data = []
        for i_idx in self.indices:
            X = self.x[i_idx]
            # X is [x,y,z], but we'll only use x,y for 2D
            vx = X[0]*scale + offset_x
            vy = X[1]*scale + offset_y
            vertex_data.extend([vx, vy])

        # color
        r255 = tuple(int(c*255) for c in self.color)

        batch.add(
            vertex_count,
            GL_TRIANGLES,
            None,
            ('v2f', vertex_data)
        )

    # FIXME : should change code to 2D version
    def external_forces(self, G, wind, DT):
        self.dv[:] = 0.0

        for j in range(self.indices.shape[0]//3):
            vi1 = self.indices[j * 3 + 0]
            vi2 = self.indices[j * 3 + 1]
            vi3 = self.indices[j * 3 + 2]

            V1 = self.x[vi1]
            V2 = self.x[vi2]
            V3 = self.x[vi3]

            Vv1 = self.v[vi1]
            Vv2 = self.v[vi2]
            Vv3 = self.v[vi3]

            # normal from cross
            n = np.cross(V2 - V1, V3 - V1)
            A = np.linalg.norm(n)*0.5
            if A < 1e-8:
                continue
            n = n/np.linalg.norm(n)

            # FL ~ KL*A*(0,0,1)? 
            # We'll keep it for structure, though this is 2D ignoring actual meaning
            FL = self.KL * A * np.array([0,0,1], dtype=np.float32)

            vavg = (Vv1 + Vv2 + Vv3)/3 + wind*DT
            if np.dot(n, vavg)<0:
                n = -n

            # FD
            # v.norm()^2 * A * ...
            # We'll replicate the code
            FD = 0.5*self.KD*(np.linalg.norm(vavg)**2)*A*np.dot(vavg, n)*(-vavg)

            for vi in (vi1, vi2, vi3):
                w = self.WEIGHT[vi]
                if w>0:
                    self.dv[vi] += (FL+FD)*DT/w

        # average out by nt
        for i in range(self.V):
            if self.nt[i]>0:
                self.dv[i]/=self.nt[i]
            # gravity
            self.dv[i][1] += (G*DT)
            # damping
            self.v[i] += self.DAMPING*self.dv[i]

    """
    p = x + v*dt
    """
    def make_predictions(self, DT):
        self.p = self.x + DT * self.v

    """
    update v, x from p
    if fixed => revert
    """
    def apply_correction(self, DT):
        for i in range(self.V):
            self.v[i] = (self.x[i] - self.p[i]) / DT
            self.x[i] = self.p[i]
            if self.fixed[i]==1:
                self.x[i] = self.POSITIONS[i]

    def fix_point(self, idx):
        self.fixed[idx] = 1
        self.WEIGHT[idx] = 0.0

    def solve_stretching_constraint(self, iterations):
        KS = self.KS**iterations

        for i in range(self.edges.shape[0]/2):
            p1 = self.edges[2 * i]
            p2 = self.edges[2 * i+1]

            d = np.linalg.norm(self.POSITIONS[p1] - self.POSITIONS[p2])
            l = np.linalg.norm(self.p[p1] - self.p[p2])
            n = (self.p[p1] - self.p[p2]) / l

            lagrange = (l - d) / 2
            
            m = self.WEIGHT[p1] + self.WEIGHT[p2]

            self.p[p1] -= KS * (self.WEIGHT[p1]/m) * lagrange * n
            self.p[p2] += KS * (self.WEIGHT[p2]/m) * lagrange * n
    
    def fit(self,a):
        return min(max(a,-1),1)

    def solve_bending_constraints(self, iterations):
        KB = self.KB**iterations

        for i in range(self.tripairs.shape[0]/4):
            p1 = self.tripairs[4 * i]
            p2 = self.tripairs[4 * i+1]
            p3 = self.tripairs[4 * i+2]
            p4 = self.tripairs[4 * i+3]

            v2, w2 = self.p[p2] - self.p[p1], self.POSITIONS[p2]-self.POSITIONS[p1]

            v3, w3 = self.p[p3] - self.p[p1], self.POSITIONS[p3]-self.POSITIONS[p1]

            v4, w4 = self.p[p4]-self.p[p1], self.POSITIONS[p4]-self.POSITIONS[p1]

            vn1, wn1 = np.cross(v2, v3), np.cross(w2, w3)
            vn1, wn1 = vn1/np.linalg.norm(vn1), wn1/np.linalg.norm(wn1)

            vn2, wn2 = np.cross(v2, v4), np.cross(w2, w4)
            vn2, wn2 = vn2/np.linalg.norm(vn2), wn2/np.linalg.norm(wn2)

            vd, wd = np.dot(vn1,vn2), np.dot(wn1,wn2)

            vd = self.fit(vd)
            wd = self.fit(wd)
            
            q3 = (np.cross(v2, vn2) + np.cross(vn1, v2) * vd) / np.linalg.norm(np.cross(v2, v3))
            q4 = (np.cross(v2, vn1) + np.cross(vn2, v2) * vd) / np.linalg.norm(np.cross(v2, v4))
            q2 = (-(np.cross(v3, vn2) + np.cross(vn1, v3) * vd) / np.linalg.norm(np.cross(v2, v3))) - (np.cross(v4, vn1) + np.cross(vn2, v4) * vd) / np.linalg.norm(np.cross(v2, v4))
            q1 = -q2 - q3 - q4

            w = acos(wd)
            
            sd = -sqrt(1 - vd*vd) * (acos(vd) - w)
            
            S = self.WEIGHT[p1] * np.linalg.norm(q1)**2 + self.WEIGHT[p2] * np.linalg.norm(q2)**2 + self.WEIGHT[p3] * np.linalg.norm(q3)**2 + self.WEIGHT[p4] * np.linalg.norm(q4)**2
            
            if not sd == 0:
                self.p[p1] += KB * self.WEIGHT[p1] * (sd/S * q1)
                self.p[p2] += KB * self.WEIGHT[p2] * (sd/S * q2)
                self.p[p3] += KB * self.WEIGHT[p3] * (sd/S * q3)
                self.p[p4] += KB * self.WEIGHT[p4] * (sd/S * q4)

    def solve_collision_constraints(self, obj):
        KC = 1.0
        for i in range(self.V):
            p = self.p[i]
            x = self.x[i]
            
            corr = obj.solve_collision_constraint(p, x)
            self.p[i] += KC * corr
            
    def solve_self_collision_constraints(self, dt):
        for i in range(self.V):
            p = self.p[i]
            x = self.x[i]

            for j in range(self.indices.shape[0]/3):
                vi1 = self.indices[j * 3 + 0]
                vi2 = self.indices[j * 3 + 1]
                vi3 = self.indices[j * 3 + 2]

                V1 = self.p[vi1]
                V2 = self.p[vi2]
                V3 = self.p[vi3]

                V1o = self.x[vi1]
                V2o = self.x[vi2]
                V3o = self.x[vi3]

                
                if not(i == vi1 or i == vi2 or i == vi3):
                    tc = (V1 + V2 + V3) / 3
                    D = np.linalg.norm(V2 - V1) + np.linalg.norm(V3 - V1)
                    if np.linalg.norm(p - tc) < D/2:
                        thickness = 0.002

                        t = self.triangle_collision(p, x, V1, V2, V3)
                        t2 = self.triangle_collision(p, x, V1o, V2o, V3o)
                        
                        if t >= 0 and t <= 1 and t2 >= 0:
                            n = np.cross(V2 - V1, V3 - V1)
                            n = n / np.linalg.norm(n)
                            
                            v = p - x
                            if np.dot(n, v) > 0:
                                n = -n

                            C = np.dot(n, p - V1) - 2 * thickness
                            M = -1. * n.outer_product(n)        # FIXME
                            M[0, 0] += 1
                            M[1, 1] += 1 
                            M[2, 2] += 1 

                            M = M / n.norm()

                            cp = n
                            c1 = np.cross(V2 - V3, M @ n) - n
                            c2 = np.cross(V3 - V1, M @ n)
                            c3 = np.cross(V2 - V1, M @ n)

                            S = self.WEIGHT[i] * np.linalg.norm(cp)**2 + self.WEIGHT[vi1] * np.linalg.norm(c1)**2 + self.WEIGHT[vi2] * np.linalg.norm(c2)**2 + self.WEIGHT[vi3] * np.linalg.norm(c3)**2

                            
                            self.p[i] -= C/S * self.WEIGHT[i] * cp
                            self.p[vi1] -= C/S * self.WEIGHT[vi1] * c1
                            self.p[vi2] -= C/S * self.WEIGHT[vi2] * c2
                            self.p[vi3] -= C/S * self.WEIGHT[vi3] * c3
