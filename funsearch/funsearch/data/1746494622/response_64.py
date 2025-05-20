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

def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v1 using a hybrid heuristic.

    Combines a nearest neighbor heuristic for initialization and a 2-opt local search for optimization.

    Returns:
        A permutation of cities that minimizes the total route distance.
    """

    # Initialize a random route
    num_cities = len(matrix_distances)
    current_city = np.random.randint(num_cities)
    route = [current_city]

    # Nearest neighbor heuristic
    for _ in range(num_cities - 1):
        nearest_city = np.argmin(matrix_distances[current_city])
        while nearest_city in route:
            nearest_city = np.argmin(matrix_distances[current_city][nearest_city+1:]) + nearest_city + 1
        route.append(nearest_city)
        current_city = nearest_city

    # 2-opt local search
    for _ in range(100):
        for i in range(num_cities):
            for j in range(i + 2, num_cities):
                distance_original = matrix_distances[route[i]][route[j]]
                distance_reversed = matrix_distances[route[i]][route[(j-1)%num_cities]] + matrix_distances[route[(j-1)%num_cities]][route[j]]
                if distance_reversed < distance_original:
                    route[i+1:j] = route[j-1:i:-1]

    return tuple(route)

def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Args:
        route: A permutation of cities.
        matrix_distances: A square matrix of distances between cities.

    Returns:
        The total distance of the route.
    """

    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]

    return total_distance
