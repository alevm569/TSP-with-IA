import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
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
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Uses a hybrid heuristic combining:
    - Nearest neighbor to generate an initial solution.
    - 2-opt neighborhood search to improve the solution.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Generate an initial solution using nearest neighbor
    start_city = np.random.randint(len(distances))
    current_city = start_city
    route = [current_city]

    while len(route) < len(distances):
        nearest_city = np.argmin(distances[current_city])
        while nearest_city in route:
            nearest_city = np.argmin(distances[current_city][nearest_city+1:]) + nearest_city + 1
        route.append(nearest_city)
        current_city = nearest_city

    # Improve the solution using 2-opt neighborhood search
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_original = distances[route[i]][route[j]] + distances[route[(j + 1) % len(route)]][route[(i + 1) % len(route)]]
            distance_reversed = distances[route[i]][route[(j + 1) % len(route)]] + distances[route[j]][route[(i + 1) % len(route)]]
            if distance_reversed < distance_original:
                route[i:j+1] = route[j:i:-1]

    return tuple(route)
