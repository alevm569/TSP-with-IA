import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(0)

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

    Uses a hybrid approach combining the nearest neighbor heuristic, the cheapest insertion heuristic,
    and the 2-opt local search optimization.
    """

    # Initialize the route using the nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(1, len(distances)))

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # Use the cheapest insertion heuristic to refine the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            if distances[route[i]][route[j]] > distances[route[i]][route[j - 1]]:
                route[i:j] = route[j - 1:i:-1]

    # Perform 2-opt local search to further optimize the route
    for _ in range(100):
        i, j = np.random.randint(0, len(route), 2)
        route[i:j+1] = route[j:i-1:-1]

    return tuple(route)
