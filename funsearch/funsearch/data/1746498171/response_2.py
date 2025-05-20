import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""
    # Implement a new heuristic or combination of heuristics here.

    # Example heuristic:
    # 1. Start from an arbitrary city.
    # 2. Find the city with the shortest distance to the current city.
    # 3. Add this city to the route.
    # 4. Repeat steps 2-3 until all cities have been visited.
    # 5. Return to the starting city.

    # Example code for the heuristic:
    route = [0]  # Start from city 0
    visited = set([0])

    for _ in range(len(_distances) - 1):
        current_city = route[-1]
        nearest_city = np.argmin(_distances[current_city][~np.isin(_distances[current_city], visited)])
        route.append(nearest_city)
        visited.add(nearest_city)

    route.append(0)  # Return to starting city
    return tuple(route)
