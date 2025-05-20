import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route_v2(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""
    # Implement a new heuristic or strategy here.
    # For example, you could use a hybrid of the nearest neighbor and cheapest insertion heuristics.
    # You may need to adjust the code based on the specific heuristic or strategy you choose.

    # Example heuristic:
    # Initialize the route with the first city.
    route = [0]
    # Visit the remaining cities in order of their distance to the last city in the route.
    for _ in range(len(_distances) - 1):
        current_city = route[-1]
        next_city = np.argmin(_distances[current_city])
        route.append(next_city)

    return tuple(route)
