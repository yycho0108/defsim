#!/usr/bin/env python3

from typing import Dict
from dataclasses import dataclass
import numpy as np


@dataclass
class ObjectState:
    # NDArray[N,2,float]
    node: np.ndarray
    # NDArray[M,2,int]
    edge: np.ndarray


SimState = Dict[str, ObjectState]
