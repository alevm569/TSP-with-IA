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
    Improved version of the find_best_route function.

    Uses a hybrid approach combining nearest neighbor and local search heuristics.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Initialize a random starting city
    current_city = np.random.randint(len(distances))

    # Create an empty list to store the route
    route = [current_city]

    # Visit each city exactly once
    remaining_cities = set(range(len(distances))) - {current_city}

    # Perform nearest neighbor heuristic to find the next city
    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Perform local search to improve the route
    funsearch.local_search(distances, route)

    return tuple(route)
