import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(0)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a hybrid approach."""

    # Perform nearest neighbor search to get an initial route
    current_city = np.random.randint(len(_distances))
    route = [current_city]

    while len(route) < len(_distances):
        closest_city = None
        min_distance = float('inf')

        for city in range(len(_distances)):
            if city not in route:
                distance = _distances[current_city][city]
                if distance < min_distance:
                    closest_city = city
                    min_distance = distance

        route.append(closest_city)
        current_city = closest_city

    # Perform local search to refine the route
    for i in range(100):
        for j in range(len(route)):
            for k in range(j + 2, len(route)):
                new_route = route[:j] + route[j+k:k:-1] + route[k+1:]
                if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                    route = new_route

    return tuple(route)
