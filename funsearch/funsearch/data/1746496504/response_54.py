import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

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
    Improved version of the find_best_route function.

    Uses a combination of nearest neighbor and local search heuristics.

    Args:
        distances: A square matrix of distances between cities.

    Returns:
        A tuple of integers representing the best route.
    """

    # Initialize a random starting city
    current_city = np.random.randint(len(distances))

    # Create an empty list to store the route
    route = [current_city]

    # Visit all cities except the starting city
    remaining_cities = set(range(len(distances)))
    remaining_cities.remove(current_city)

    # Nearest neighbor heuristic
    while remaining_cities:
        # Find the city with the shortest distance from the current city
        nearest_city = min(remaining_cities, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Local search
    for _ in range(100):  # Perform 100 iterations of local search
        i, j = np.random.randint(len(route), size=2)
        route[i], route[j] = route[j], route[i]

    return tuple(route)
