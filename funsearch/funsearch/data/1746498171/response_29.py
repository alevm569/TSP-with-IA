import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a hybrid approach that combines the nearest neighbor and cheapest insertion heuristics.

    # Ensure reproducibility of random number generation.
    np.random.seed(42)

    # Define a new objective function that includes the new heuristic.
    def objective(route):
        distance = calculate_route_distance(route, _distances)
        # Add a penalty for invalid routes.
        for i in range(len(route)):
            if route[i] in route[:i]:
                distance += 1000
        return distance

    # Use a metaheuristic algorithm to find the best route.
    best_route = funsearch.minimize(objective, len(_distances))

    return best_route
