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
    """Improved version of `find_best_route_v0`."""

    # Use a hybrid heuristic combining nearest neighbor and cheapest insertion.
    # Initialize a random starting city.
    np.random.seed(0)
    start_city = np.random.randint(len(_distances))
    route = [start_city]

    # Use nearest neighbor to find the next city in the route.
    current_city = start_city
    while len(route) < len(_distances):
        next_city = np.argmin(_distances[current_city])
        if next_city not in route:
            route.append(next_city)
            current_city = next_city

    # Use cheapest insertion to refine the route.
    while True:
        min_distance = np.inf
        for i in range(len(_distances)):
            if i not in route:
                distance = _distances[route[-1]][i]
                if distance < min_distance:
                    min_distance = distance
                    best_city = i

        if min_distance == np.inf:
            break

        route.append(best_city)

    return tuple(route)
