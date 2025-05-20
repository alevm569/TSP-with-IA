import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
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

    We will use a hybrid approach combining the nearest neighbor and k-opt heuristics.
    """

    # Generate an initial route using nearest neighbor
    start_city = np.random.randint(len(matrix_distances))
    current_city = start_city
    route = [current_city]
    remaining_cities = set(range(len(matrix_distances)))
    remaining_cities.remove(current_city)

    while remaining_cities:
        next_city = min(remaining_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(next_city)
        remaining_cities.remove(next_city)
        current_city = next_city

    # Perform k-opt optimization to improve the route
    k = 2  # Number of cities to swap at a time
    for _ in range(100):  # Number of iterations
        for i in range(len(route)):
            for j in range(i + k, len(route)):
                route = funsearch.k_opt(route, matrix_distances, k)

    return tuple(route)
