import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Apply ACO (ant colony optimization) algorithm
    aco = funsearch.ACO(distance_matrix=_distances)
    best_route = aco.run()

    # Ensure the route includes all cities exactly once and returns to the starting point
    assert len(set(best_route)) == len(_distances)
    assert best_route[0] == best_route[-1]

    return best_route
