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


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2.

    Uses a hybrid heuristic combining nearest neighbor and 2-opt operations.
    """
    # Initialize a random starting city
    start_city = np.random.randint(len(distances))
    current_city = start_city
    route = [current_city]

    # Generate the initial route using the nearest neighbor heuristic
    for _ in range(len(distances) - 1):
        nearest_city = np.argmin(distances[current_city])
        while nearest_city in route:
            nearest_city = np.argmin(distances[current_city][nearest_city + 1:]) + nearest_city + 1
        route.append(nearest_city)
        current_city = nearest_city

    # Apply 2-opt operations to improve the route
    for _ in range(100):
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                distance_original = distances[route[i]][route[j]]
                distance_reversed = distances[route[i]][route[j - 1]] + distances[route[j]][route[i]]
                if distance_reversed < distance_original:
                    route = route[:i] + route[j:i:-1] + route[j + 1:]

    return tuple(route)
