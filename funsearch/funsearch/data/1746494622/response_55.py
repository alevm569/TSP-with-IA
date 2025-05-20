import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Apply simulated annealing with 1000 iterations and a cooling schedule
    best_route = funsearch.simulated_annealing(
        funsearch.nearest_neighbor,
        funsearch.tabu_search,
        funsearch.local_search,
        funsearch.k_opt,
        iterations=1000,
        cooling_schedule=funsearch.geometric_cooling,
        distances=_distances,
    )

    return best_route
