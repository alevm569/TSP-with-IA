import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` with added heuristics."""

    # Use hybrid heuristic combining nearest neighbor and 2-opt
    def hybrid_heuristic(distances):
        # Nearest neighbor heuristic
        current_city = np.random.randint(len(distances))
        route = [current_city]

        while len(route) < len(distances):
            next_city = np.argmin([distances[current_city][j] for j in range(len(distances)) if j not in route])
            route.append(next_city)
            current_city = next_city

        # 2-opt heuristic
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                distance_difference = distances[route[i]][route[(j - 1)]] + distances[route[j]][route[(i + 1)]] - distances[route[i]][route[j]] - distances[route[(j - 1)]]][route[i]]
                if distance_difference < 0:
                    route[i], route[j] = route[j], route[i]

        return tuple(route)

    # Use hybrid heuristic to find the best route
    best_route = hybrid_heuristic(_distances)

    # Return the best route
    return best_route
