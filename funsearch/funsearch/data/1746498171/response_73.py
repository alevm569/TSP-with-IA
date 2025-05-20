import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(0)

def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO heuristic."""

    # Initialize ACO algorithm
    aco = funsearch.ACO(distance_matrix=_distances)

    # Run ACO algorithm for a specified number of iterations
    best_route = aco.run(max_iter=1000)

    return best_route
