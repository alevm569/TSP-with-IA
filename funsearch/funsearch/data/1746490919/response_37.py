import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix


def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with new heuristics."""

    # Set a seed for reproducibility
    np.random.seed(42)

    # Use a hybrid heuristic combining local search with a 2-opt optimization
    best_route = funsearch.optimize.hybrid_search(
        _distances,
        funsearch.heuristics.local_search,
        funsearch.heuristics.two_opt,
        iterations=1000,
        population_size=100,
    )

    return best_route
