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
    Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Strategy:
    - Use a hybrid approach combining nearest neighbor and 2-opt heuristics.
    - Start with the nearest neighbor permutation.
    - Apply 2-opt swaps iteratively until no further improvements are found.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route with the nearest neighbor permutation
    start_city = np.random.randint(len(distances))
    route = [start_city]
    remaining_cities = set(range(len(distances))) - {start_city}

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # Apply 2-opt swaps iteratively
    improved = True
    while improved:
        improved = False
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                distance_difference = distances[route[i]][route[j]] - distances[route[(j - 1) % len(route)]][route[(j + 1) % len(route)]]
                if distance_difference < 0:
                    route[i:j+1] = route[j:i:-1]
                    improved = True

    return tuple(route)
