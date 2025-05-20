import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with ACO heuristic."""

    # Create ACO algorithm
    aco = funsearch.ACO(distances=_distances)

    # Run ACO algorithm to find the best route
    best_route = aco.run()

    return best_route
