import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Set a seed for reproducibility
    np.random.seed(42)

    # Use a hybrid approach combining nearest neighbor and 2-opt heuristics
    def hybrid_heuristic(distances):
        # Start with the nearest neighbor heuristic
        route = funsearch.nearest_neighbor(distances)

        # Perform 2-opt local search to refine the route
        route = funsearch.two_opt(route, distances)

        return route

    # Find the best route using the hybrid heuristic
    best_route = hybrid_heuristic(_distances)

    return best_route
