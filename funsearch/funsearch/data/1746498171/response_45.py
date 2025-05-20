import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a hybrid approach combining the nearest neighbor and cheapest insertion heuristics
    # Perform local search with a k-opt heuristic to refine the solution
    # Use a seed for reproducibility of the random number generator

    np.random.seed(42)  # Set a seed for reproducibility

    # Initialize the route using the nearest neighbor heuristic
    start_city = np.random.randint(len(_distances))
    route = [start_city]
    unvisited = list(range(len(_distances)))
    unvisited.remove(start_city)

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Perform local search with a k-opt heuristic
    def k_opt(route: list, k: int) -> list:
        # Implement a k-opt heuristic here
        pass

    for _ in range(100):  # Perform 100 iterations of local search
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                route = k_opt(route, k=2)  # Perform a 2-opt move

    return tuple(route)
