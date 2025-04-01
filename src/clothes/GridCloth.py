# grid_cloth.py

import numpy as np
from .cloth import Cloth

class GridCloth(Cloth):
    """
    2D cloth in a grid arrangement, similar to the old GridCloth code.
    """

    def __init__(
        self,
        N=(64,64),
        S=(1,1),
        W=1.0,
        KS=0.6,
        KB=0.4,
        KC=0.4,
        KD=0.0,
        KL=0.0,
        DAMPING=0.9,
        center=(0.0,0.0,0.0),
        color=(1,1,1)
    ):
        """
        N: (nx, ny)
        S: (size_x, size_y)
        W: total weight, distributed among vertices
        KS, KB, KC, KD, KL, DAMPING as in the old code
        center: cloth center in (x,y,z) - default z=0
        color: (r,g,b)
        """
        self.nx, self.ny = N
        self.Sx, self.Sy = S
        self.center = np.array(center, dtype=np.float32)

        self.W = W
        self.KS = KS
        self.KB = KB
        self.KC = KC
        self.KD = KD
        self.KL = KL
        self.DAMPING = DAMPING
        self.color = color

        # build Nx x Ny grid of positions
        # store in Nx*Ny x 3
        pos_2d, vel_2d = self._make_grid()
        # flatten
        positions = pos_2d.reshape((-1,3))
        velocities = vel_2d.reshape((-1,3))

        # build indices
        indices = self._make_indices()
        # build edges
        edges = self._make_edges()
        # build tripairs
        tripairs = self._make_tripairs()

        # build weight array
        weight = np.ones((self.nx*self.ny,), dtype=np.float32)*(W/(self.nx*self.ny))

        # init super
        super().__init__(
            V=self.nx*self.ny,
            POSITIONS=positions,
            VELOCITIES=velocities,
            WEIGHT=weight,
            indices=indices,
            edges=edges,
            tripairs=tripairs,
            color=color,
            KS=KS,
            KC=KC,
            KB=KB,
            KD=KD,
            KL=KL,
            DAMPING=DAMPING
        )

    def _make_grid(self):
        """
        returns pos, vel, shape (nx, ny, 3)
        """
        pos = np.zeros((self.nx, self.ny, 3), dtype=np.float32)
        vel = np.zeros((self.nx, self.ny, 3), dtype=np.float32)
        dx = self.Sx/(self.nx-1) if self.nx>1 else 0.0
        dy = self.Sy/(self.ny-1) if self.ny>1 else 0.0

        for i in range(self.nx):
            for j in range(self.ny):
                x = self.center[0] + (i*dx - self.Sx*0.5)
                y = self.center[1] + (j*dy - self.Sy*0.5)
                pos[i,j,0] = x
                pos[i,j,1] = y
                pos[i,j,2] = self.center[2] # default 0
                vel[i,j] = (0,0,0)
        return pos, vel

    def _make_indices(self):
        # (nx-1)*(ny-1)*2 tri, each tri => 3 idx => total # => ...
        tri_count = (self.nx-1)*(self.ny-1)*2
        indices = np.zeros((tri_count*3,), dtype=np.int32)

        idx = 0
        for i in range(self.nx):
            for j in range(self.ny):
                if i<self.nx-1 and j<self.ny-1:
                    square_id = (i*(self.ny-1))+ j
                    # 1st tri
                    i0 = i*self.ny + j
                    i1 = (i+1)*self.ny + j
                    i2 = i*self.ny + (j+1)
                    indices[idx*3+0] = i0
                    indices[idx*3+1] = i1
                    indices[idx*3+2] = i2
                    idx+=1
                    # 2nd tri
                    i3 = (i+1)*self.ny + (j+1)
                    i4 = i*self.ny + (j+1)
                    i5 = (i+1)*self.ny + j
                    indices[idx*3+0] = i3
                    indices[idx*3+1] = i4
                    indices[idx*3+2] = i5
                    idx+=1

        return indices

    def _make_edges(self):
        """
        replicate the logic:
          if i+1<nx => edge
          if j+1<ny => edge
          plus diagonal edges if needed
        store flatten in pairs
        """
        edges_list = []
        for i in range(self.nx):
            for j in range(self.ny):
                if i+1<self.nx:
                    edges_list.append(i*self.ny+j)
                    edges_list.append((i+1)*self.ny+j)
                if j+1<self.ny:
                    edges_list.append(i*self.ny+j)
                    edges_list.append(i*self.ny+(j+1))
                # diagonal?
                if i+1<self.nx and j+1<self.ny:
                    edges_list.append(i*self.ny+j)
                    edges_list.append((i+1)*self.ny+(j+1))

                    edges_list.append(i*self.ny+(j+1))
                    edges_list.append((i+1)*self.ny+j)

        return np.array(edges_list, dtype=np.int32)

    def _make_tripairs(self):
        """
        replicate old code logic for bending
        storing 4 points p1,p2,p3,p4 in sets of 4
        ...
        We'll do a minimal approach or replicate logic if needed
        """
        tripair_list=[]
        for i in range(self.nx):
            for j in range(self.ny):
                if i+1<self.nx and j+1<self.ny:
                    p1 = i*self.ny+j
                    p2 = (i+1)*self.ny+(j+1)
                    p3 = i*self.ny+(j+1)
                    p4 = (i+1)*self.ny+j
                    tripair_list.extend([p1,p2,p3,p4])
                # more logic for neighbors if needed ...
        return np.array(tripair_list, dtype=np.int32)

    def _make_weight(self):
        """
        if needed, we do it in the super class
        """
        pass

