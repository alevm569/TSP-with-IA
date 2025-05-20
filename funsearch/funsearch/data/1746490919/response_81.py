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


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Use the nearest neighbor heuristic to generate an initial route.
    Then, perform local search using the 2-opt heuristic to improve the route.
    """

    # Generate an initial route using the nearest neighbor heuristic
    num_cities = matrix_distances.shape[0]
    current_city = np.random.randint(num_cities)
    route = [current_city]

    while len(route) < num_cities:
        nearest_city = np.argmin(matrix_distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Perform local search using the 2-opt heuristic
    for _ in range(100):
        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                distance_before = matrix_distances[route[i]][route[(i + 1) % num_cities]] + \
                                matrix_distances[route[j]][route[(j + 1) % num_cities]]
                distance_after = matrix_distances[route[i]][route[j]] + \
                                matrix_distances[route[(i + 1) % num_cities]][route[(j + 1) % num_cities]]

                if distance_after < distance_before:
                    route[i + 1:j + 1] = route[j:i:-1]

    return tuple(route)
