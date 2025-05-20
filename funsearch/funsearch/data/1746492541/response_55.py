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

    Strategy:
    - Use a hybrid approach that combines the nearest neighbor and cheapest insertion heuristics.
    - Employ a local search algorithm (2-opt) to refine the initial solution.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route using the nearest neighbor heuristic
    start_city = np.random.randint(len(distances))
    route = [start_city]
    remaining_cities = set(range(len(distances))) - {start_city}

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # Refine the route using 2-opt local search
    funsearch.optimize.local_search(route, distances, method='2-opt')

    return tuple(route)
