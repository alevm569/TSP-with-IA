import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`.

    Uses a hybrid approach combining nearest neighbor and 2-opt heuristics.
    """
    # Nearest neighbor heuristic
    start_city = np.random.randint(len(_distances))
    route = [start_city]
    unvisited = set(range(len(_distances)))
    unvisited.remove(start_city)

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # 2-opt heuristic
    for _ in range(100):  # Number of iterations
        for i in range(1, len(route)):
            for j in range(i + 1, len(route)):
                if _distances[route[i - 1]][route[j]] + _distances[route[i]][route[j - 1]] < _distances[route[i - 1]][route[i]] + _distances[route[j]][route[j - 1]]:
                    route[i:j+1] = route[j:i-1:-1]

    return tuple(route)
