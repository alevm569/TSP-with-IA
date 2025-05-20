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
    Improved version of `find_best_route_v2` using a hybrid heuristic.
    """

    # Nearest neighbor heuristic
    current_city = 0
    route = [current_city]

    while len(route) < len(_distances):
        nearest_city = np.argmin(_distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Local search with perturbation
    best_route = route
    best_distance = calculate_route_distance(best_route, _distances)

    for _ in range(100):
        perturbed_route = funsearch.perturb(best_route)
        perturbed_distance = calculate_route_distance(perturbed_route, _distances)

        if perturbed_distance < best_distance:
            best_route = perturbed_route
            best_distance = perturbed_distance

    return best_route
