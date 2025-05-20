import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize route with starting city
    route = [0]

    # Create a list of unvisited cities
    unvisited = list(range(1, len(_distances)))

    # Use a greedy heuristic to find the next city
    while unvisited:
        current = route[-1]
        next_city = unvisited[np.argmin([_distances[current][city] for city in unvisited])]
        route.append(next_city)
        unvisited.remove(next_city)

    # Return to the starting city
    route.append(0)

    return tuple(route)
