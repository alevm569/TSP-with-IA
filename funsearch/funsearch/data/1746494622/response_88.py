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

    best_route = find_best_route_v1(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`.

    Uses a hybrid approach combining nearest neighbor and local search heuristics.
    """

    # Initialize a random starting route
    np.random.seed(42)  # Ensure reproducibility
    route = np.random.permutation(np.arange(_distances.shape[0]))

    # Use nearest neighbor to fill in missing cities
    current_city = route[0]
    for i in range(1, len(route)):
        nearest_city = np.argmin(_distances[current_city])
        route[i] = nearest_city
        current_city = nearest_city

    # Perform local search to optimize the route
    for _ in range(100):
        # Randomly swap two cities in the route
        i, j = np.random.randint(len(route), size=2)
        route[i], route[j] = route[j], route[i]

        # Check if the new route is better
        if calculate_route_distance(route, _distances) < calculate_route_distance(route, _distances):
            pass  # Keep the new route
        else:
            route[i], route[j] = route[j], route[i]  # Restore the previous route

    return route
