import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
import matplotlib.patches as patches

# ---------------------------
# Collision Object Classes
# ---------------------------
class CollisionObject:
    def resolve_collision(self, pos):
        """Return corrected position for a given particle position.
        Should be overridden by subclasses."""
        return pos

class Sphere(CollisionObject):
    def __init__(self, center, radius):
        self.center = np.array(center, dtype=float)
        self.radius = radius

    def resolve_collision(self, pos):
        v = pos - self.center
        dist = np.linalg.norm(v)
        if dist < self.radius:
            # If particle is exactly at the center, push arbitrarily to the right.
            if dist == 0:
                v = np.array([1.0, 0.0])
                dist = 1.0
            pos = self.center + v / dist * self.radius
        return pos

class Plane(CollisionObject):
    def __init__(self, point, normal):
        self.point = np.array(point, dtype=float)
        # Ensure the normal is unit length.
        self.normal = np.array(normal, dtype=float)
        self.normal /= np.linalg.norm(self.normal)

    def resolve_collision(self, pos):
        # For this plane, assume a collision happens when pos is "below" the plane.
        # Calculate signed distance from pos to the plane.
        d = np.dot(pos - self.point, self.normal)
        if d < 0:
            # Project pos onto the plane.
            pos = pos - d * self.normal
        return pos

# ---------------------------
# Cloth Simulation Class
# ---------------------------
class Cloth:
    def __init__(self, num_x, num_y, spacing, origin):
        self.num_x = num_x
        self.num_y = num_y
        self.spacing = spacing
        self.origin = np.array(origin, dtype=float)
        # Initialize particle positions and previous positions (for Verlet integration)
        self.positions = np.zeros((num_x, num_y, 2), dtype=float)
        self.prev_positions = np.zeros_like(self.positions)
        for i in range(num_x):
            for j in range(num_y):
                pos = self.origin + np.array([i * spacing, -j * spacing])
                self.positions[i, j] = pos
                self.prev_positions[i, j] = pos
        # Build distance constraints between neighboring particles.
        self.constraints = []
        self.build_constraints()

    def build_constraints(self):
        for i in range(self.num_x):
            for j in range(self.num_y):
                if i < self.num_x - 1:
                    p1 = self.positions[i, j]
                    p2 = self.positions[i+1, j]
                    rest_len = np.linalg.norm(p2 - p1)
                    self.constraints.append(((i, j), (i+1, j), rest_len))
                if j < self.num_y - 1:
                    p1 = self.positions[i, j]
                    p2 = self.positions[i, j+1]
                    rest_len = np.linalg.norm(p2 - p1)
                    self.constraints.append(((i, j), (i, j+1), rest_len))
                # Diagonal constraints for shape preservation
                if i < self.num_x - 1 and j < self.num_y - 1:
                    p1 = self.positions[i, j]
                    p2 = self.positions[i+1, j+1]
                    rest_len = np.linalg.norm(p2 - p1)
                    self.constraints.append(((i, j), (i+1, j+1), rest_len))
                if i < self.num_x - 1 and j > 0:
                    p1 = self.positions[i, j]
                    p2 = self.positions[i+1, j-1]
                    rest_len = np.linalg.norm(p2 - p1)
                    self.constraints.append(((i, j), (i+1, j-1), rest_len))

    def verlet_integration(self, dt, gravity):
        """Predict new positions using a Verlet-like integration."""
        temp = self.positions.copy()
        self.positions = self.positions + (self.positions - self.prev_positions) + gravity * dt**2
        self.prev_positions = temp

    def satisfy_constraints(self, iterations, collision_objects):
        """Project constraints and resolve collisions iteratively."""
        for _ in range(iterations):
            # Enforce distance constraints.
            for (i1, j1), (i2, j2), rest_len in self.constraints:
                p1 = self.positions[i1, j1]
                p2 = self.positions[i2, j2]
                delta = p2 - p1
                d = np.linalg.norm(delta)
                if d == 0:
                    continue
                diff = (d - rest_len) / d
                correction = 0.5 * delta * diff
                self.positions[i1, j1] += correction
                self.positions[i2, j2] -= correction

            # Resolve collisions for each particle using the provided collision objects.
            for i in range(self.num_x):
                for j in range(self.num_y):
                    for obj in collision_objects:
                        self.positions[i, j] = obj.resolve_collision(self.positions[i, j])

# ---------------------------
# Simulation Setup and Loop
# ---------------------------
# Simulation parameters
dt = 0.016
gravity = np.array([0, -9.81])
constraint_iters = 5

# Create a cloth object.
cloth = Cloth(num_x=20, num_y=20, spacing=0.05, origin=[-0.5, 0.8])

# Define collision objects.
spheres = [
    Sphere(center=[0.0, -0.2], radius=0.15),
    Sphere(center=[-0.2, -0.2], radius=0.3),
]

# For example, define a horizontal plane at y = -0.8 with an upward normal.
plane = Plane(point=[0.0, -0.8], normal=[0, 1])
# You can combine as many collision objects as you like.
collision_objects = [*spheres, plane]

def simulation_step():
    cloth.verlet_integration(dt, gravity)
    cloth.satisfy_constraints(constraint_iters, collision_objects)

# ---------------------------
# Visualization with Matplotlib
# ---------------------------
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(-0.6, 1.2)
ax.set_ylim(-1.0, 1.0)
ax.set_aspect('equal')
ax.set_title("Reusable 2D Cloth Simulation with Collision Objects")

# Scatter plot for cloth particles.
scatter = ax.scatter(cloth.positions[:,:,0].flatten(), cloth.positions[:,:,1].flatten(), c='red', s=10)
# Line collection for cloth constraints.
lc = LineCollection([], colors='blue', linewidths=1)
ax.add_collection(lc)

# Add a circle patch for the sphere.
for sphere in spheres:
    sphere_patch = patches.Circle(sphere.center, sphere.radius, fill=False, edgecolor='black', linewidth=2)
    ax.add_patch(sphere_patch)

# Draw the plane as a horizontal line.
plane_line = plt.Line2D([-10, 10], [plane.point[1], plane.point[1]], color='green', linewidth=2)
ax.add_line(plane_line)

def update(frame):
    simulation_step()
    pts = cloth.positions.reshape(-1, 2)
    scatter.set_offsets(pts)
    segments = []
    for (i1, j1), (i2, j2), _ in cloth.constraints:
        segments.append([cloth.positions[i1, j1].tolist(), cloth.positions[i2, j2].tolist()])
    lc.set_segments(segments)
    return scatter, lc, sphere_patch, plane_line

ani = FuncAnimation(fig, update, frames=300, interval=16, blit=True)
plt.show()