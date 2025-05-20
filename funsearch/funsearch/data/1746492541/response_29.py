import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

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
    Improved version of find_best_route_v2.

    Uses a combination of nearest neighbor and 2-opt heuristics with a local search component.
    """

    # Generate an initial route using nearest neighbor
    num_cities = len(matrix_distances)
    current_city = np.random.randint(num_cities)
    route = [current_city]

    while len(route) < num_cities:
        next_city = np.argmin([matrix_distances[current_city][j] for j in range(num_cities) if j not in route])
        route.append(next_city)
        current_city = next_city

    # Perform local search using 2-opt heuristic
    for _ in range(100):  # Number of iterations
        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                distance_before = matrix_distances[route[i]][route[(i + 1) % num_cities]] + \
                                  matrix_distances[route[j]][route[(j + 1) % num_cities]]
                distance_after = matrix_distances[route[i]][route[j]] + \
                                   matrix_distances[route[(i + 1) % num_cities]][route[(j + 1) % num_cities]]
                if distance_after < distance_before:
                    route[i + 1:j + 1] = route[j:i:-1]

    return tuple(route)
