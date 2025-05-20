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

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2`.

    Uses a hybrid heuristic combining nearest neighbor and 2-opt operations.
    """

    # Initialize a random starting city
    current_city = np.random.randint(len(matrix_distances))

    # Create an empty route
    route = [current_city]

    # Perform nearest neighbor search to find the next city
    for _ in range(len(matrix_distances) - 1):
        nearest_city = np.argmin(matrix_distances[current_city])
        route.append(nearest_city)
        current_city = nearest_city

    # Perform 2-opt operations to improve the route
    for _ in range(len(matrix_distances)):
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                if calculate_route_distance(route, matrix_distances) > calculate_route_distance(route[i:j][::-1] + route[:i] + route[j:], matrix_distances):
                    route = route[i:j][::-1] + route[:i] + route[j:]

    return tuple(route)
