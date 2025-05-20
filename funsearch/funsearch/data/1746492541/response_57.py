import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using hybrid heuristic."""
    # Initialize random permutation of cities
    cities = np.random.permutation(len(_distances))

    # Perform local search optimization using 2-opt heuristic
    for _ in range(100):
        # Randomly select two cities
        i, j = np.random.randint(len(cities), size=2)
        # Swap the two cities in the permutation
        cities[i], cities[j] = cities[j], cities[i]

        # Calculate the total distance of the route
        total_distance = calculate_route_distance(cities, _distances)

        # If the new route is better, keep it
        if total_distance < calculate_route_distance(cities, _distances):
            continue

    return tuple(cities)
