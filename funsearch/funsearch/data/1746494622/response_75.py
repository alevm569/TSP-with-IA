import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with additional heuristic and randomization."""

    # Set a seed for reproducibility of randomness
    np.random.seed(42)

    # Use a hybrid heuristic that combines nearest neighbor and k-opt
    best_route = funsearch.hybrid(
        funsearch.nearest_neighbor(_distances),
        funsearch.k_opt(_distances, k=2),
    )

    return best_route
