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

def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 using a hybrid approach.

    Parameters:
    _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Uses a combination of the following heuristics:
        - Nearest neighbor for initial route construction
        - Cheapest insertion for route refinement
        - Local search with k-opt and 2-opt for route optimization

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initial route construction using nearest neighbor heuristic
    num_cities = len(_distances)
    current_city = np.random.randint(num_cities)
    route = [current_city]

    while len(route) < num_cities:
        nearest_city = np.argmin(_distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Route refinement using cheapest insertion heuristic
    for i in range(num_cities):
        best_distance = np.inf
        best_city = None

        for j in range(num_cities):
            if j not in route:
                distance = _distances[route[-1]][j]
                if distance < best_distance:
                    best_distance = distance
                    best_city = j

        route.append(best_city)

    # Route optimization using local search with k-opt and 2-opt heuristics
    funsearch.local_search(route, _distances, [funsearch.k_opt, funsearch.two_opt])

    return tuple(route)
