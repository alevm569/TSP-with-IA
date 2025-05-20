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

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a hybrid heuristic approach.

    Hybrid heuristic combines elements from the following heuristics:
        - Nearest neighbor
        - Cheapest insertion
        - Local search

    Routes must include all cities exactly once and return to the starting point.
    """

    # Nearest neighbor heuristic to generate an initial route
    start_city = np.random.randint(len(distances))
    route = [start_city]
    unvisited_cities = set(range(len(distances))) - {start_city}

    while unvisited_cities:
        current_city = route[-1]
        nearest_city = min(unvisited_cities, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # Cheapest insertion heuristic to improve the route
    for _ in range(len(distances)):
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                if distances[route[i]][route[j]] > distances[route[i]][route[j - 1]]:
                    route[i], route[j] = route[j], route[i]

    # Local search heuristic to refine the route
    for _ in range(100):
        i, j = np.random.randint(0, len(route), size=2)
        route[i], route[j] = route[j], route[i]

    return tuple(route)
