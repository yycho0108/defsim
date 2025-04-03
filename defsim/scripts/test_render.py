#!/usr/bin/env python3

from dataclasses import dataclass
import numpy as np

from infra.config import oc_cli, zen_cli
from infra.common import ObjectState, SimState
from infra.render import Config, simulate


@dataclass
class AppConfig(Config):
    seed: int = 0
    num_node: int = 4
    num_edge: int = 9
    add_ground: bool = True


@zen_cli
def main(cfg: AppConfig = AppConfig()):
    np.random.seed(cfg.seed)

    def generate_initial_state():
        state = ObjectState(
            node=np.random.uniform(size=(cfg.num_node, 2)),
            edge=np.random.randint(cfg.num_node, size=(cfg.num_edge, 2))
        )
        return state

    def reset():
        """ Dummy initialization function """
        state = dict(objA=generate_initial_state(),
                     objB=generate_initial_state())
        return state

    def step(state: SimState, in_place: bool = True) -> SimState:
        """ Dummy state transition function """
        if not in_place:
            state = {k: np.copy(v) for (k, v) in state.items()}

        # NOTE(ycho): dummy transition function
        for k, v in state.items():
            v.node += 0.01 * np.random.normal(size=v.node.shape)
        return state
    return simulate(cfg, reset, step)


if __name__ == '__main__':
    main()
