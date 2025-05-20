import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with hybrid heuristics."""

    # Generate an initial route using the nearest neighbor heuristic
    current_city = np.random.randint(len(_distances))
    route = [current_city]
    remaining_cities = set(range(len(_distances))) - {current_city}

    while remaining_cities:
        next_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(next_city)
        remaining_cities.remove(next_city)
        current_city = next_city

    # Perform local search to refine the route
    for i in range(100):
        best_route = route
        best_distance = calculate_route_distance(route, _distances)

        for j in range(len(route)):
            for k in range(j + 1, len(route)):
                new_route = route[:j] + route[k:j:-1] + route[k+1:]
                new_distance = calculate_route_distance(new_route, _distances)

                if new_distance < best_distance:
                    best_route = new_route
                    best_distance = new_distance

        if best_distance == calculate_route_distance(route, _distances):
            break
        else:
            route = best_route

    return tuple(route)
