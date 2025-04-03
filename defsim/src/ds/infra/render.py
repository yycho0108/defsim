#!/usr/bin/env python3

from typing import Tuple, Callable, Optional
from dataclasses import dataclass, replace
import numpy as np
from functools import partial
import pyglet
from pyglet import shapes

from ds.infra.common import ObjectState, SimState


@dataclass
class Config:
    """ Renderer Config. """
    win_width: int = 960
    win_height: int = 540
    # NOTE(ycho): `node_color` is currently overridden internally,
    # based on a randomly generated color per object key in SimState.
    node_color: Tuple[float, float, float] = (50, 30, 225)
    node_size: int = 10
    # NOTE(ycho): `edge_color` is currently overridden internally,
    # based on a randomly generated color per object key in SimState.
    edge_color: Tuple[float, float, float] = (255, 255, 255)
    edge_width: int = 1
    fps: float = 100.0
    # NOTE(ycho): display of the ground plane is only useful
    # if the simulation does not takes place XY (flat)-plane.
    add_ground: bool = False
    ground_height: float = 0.05


def draw_object(cfg: Config,
                batch: pyglet.graphics.Batch,
                state: ObjectState,
                pixel_from_world=lambda x: x):
    node_color = [int(x) for x in cfg.node_color]
    edge_color = [int(x) for x in cfg.edge_color]

    circles = []

    # TODO(ycho): consider applying an affine transform over node coordinates.
    coord = pixel_from_world(state.node)

    for x, y in coord:
        circle = shapes.Circle(x, y, cfg.node_size,
                               color=node_color,
                               batch=batch)
        circles.append(circle)

    edges = []
    for i, j in state.edge:
        edge = shapes.Line(*coord[i],
                           *coord[j],
                           width=cfg.edge_width,
                           color=edge_color,
                           batch=batch)
        edges.append(edge)

    return (circles, edges)


class Renderer:
    """
    Simple pyglet-based renderer for particles and their connectivities.
    """

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.batch = {}
        self.nodes = {}
        self.edges = {}
        self.colors = {}

    def _pixel_from_world(self,
                          coord_world: np.ndarray) -> np.ndarray:
        """
        Map the physical "world" coordinates to the window coordinates.
        By default, maps ([0,1], [0,1]) to ([0,W], [0,H]) in both axes.
        """
        cfg = self.cfg
        xy = np.copy(coord_world)

        # Ensure ground is displayed in the viewport.
        if cfg.add_ground:
            xy[..., 1] += cfg.ground_height
        return coord_world * [self.cfg.win_width, self.cfg.win_height]

    def __call__(self, state: SimState):
        cfg = self.cfg

        # add ground plane ...?
        if cfg.add_ground:
            if '__ground__' not in self.batch:
                self.batch['__ground__'] = pyglet.graphics.Batch()
            batch = self.batch['__ground__']
            w, h = cfg.win_width, cfg.win_height
            self.ground = shapes.Rectangle(0, 0,
                                           w, cfg.ground_height * h,
                                           batch=batch,
                                           color=(0, 255, 128))

        for k in state.keys():
            if k not in self.batch:
                self.batch[k] = pyglet.graphics.Batch()

            if k in self.nodes:
                coord = self._pixel_from_world(state[k].node)

                # Update primitives' data
                for q, n in zip(self.nodes[k], coord):
                    q.position = n
                # NOTE(ycho): is this guaranteed to be OK?
                # What if connectivities change mid-simulation?
                for q, (i, j) in zip(self.edges[k], state[k].edge):
                    q.position = coord[[i, j]].reshape(4)
            else:
                # Create primitives' data
                color = np.random.uniform(255, size=3)
                cfg = replace(self.cfg,
                              node_color=color,
                              edge_color=color)
                self.nodes[k], self.edges[k] = draw_object(
                    cfg, self.batch[k], state[k],
                    pixel_from_world=self._pixel_from_world
                )

    def draw(self):
        for k, v in self.batch.items():
            v.draw()


def simulate(
        cfg: Config,
        reset_fn: Callable[[], SimState],
        step_fn: Callable[[SimState, Optional[bool]], SimState]):
    """
    Thin pyglet application that wraps `Renderer`.
    """
    win = pyglet.window.Window(cfg.win_width,
                               cfg.win_height)
    state = reset_fn()

    # Initialize renderer.
    renderer = Renderer(cfg)
    renderer(state)

    @win.event
    def on_draw():
        win.clear()
        renderer.draw()

    def update(state, renderer, *args):
        # Update renderer.
        # if `in_place` is supported:
        state.update(step_fn(state, in_place=True))
        # else:
        # state.update(step_fn(state))
        renderer(state)

    _update = partial(update, state, renderer)
    pyglet.clock.schedule_interval(_update,
                                   1 / cfg.fps)
    pyglet.app.run()
