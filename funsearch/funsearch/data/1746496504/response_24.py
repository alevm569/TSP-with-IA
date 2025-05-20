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

def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Finds the best route using a hybrid heuristic that combines nearest neighbor and 2-opt.

    Parameters:
        distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
        A permutation of cities that minimizes the total route distance, including the return to the starting city.
    """

    # Initialize the route using the nearest neighbor heuristic
    start_city = np.random.randint(len(distances))
    route = [start_city]
    remaining_cities = set(range(len(distances))) - {start_city}

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda c: distances[current_city][c])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # Apply 2-opt to improve the route
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            distance_with_swap = distances[route[i]][route[j]] + distances[route[(j - 1) % len(route)]][route[(j + 1) % len(route)]] - distances[route[i]][route[(j - 1) % len(route)]] - distances[route[j]][route[(j + 1) % len(route)]]
            if distance_with_swap < 0:
                route[i+1:j] = route[j:i:-1]

    return tuple(route)
