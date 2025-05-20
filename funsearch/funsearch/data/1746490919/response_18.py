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
    Finds the best route using a hybrid heuristic combining nearest neighbor and 2-opt.

    Args:
        distances (np.ndarray): A square matrix of distances between cities.

    Returns:
        A tuple representing the best route.
    """

    # Initialize a random starting city
    current_city = np.random.randint(len(distances))

    # Create an empty route
    route = [current_city]

    # Use nearest neighbor to find the next city
    while len(route) < len(distances):
        nearest_city = np.argmin(distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Use 2-opt to improve the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_before = distances[route[i]][route[j]]
            route[i], route[j] = route[j], route[i]
            distance_after = distances[route[i]][route[j]]
            if distance_after < distance_before:
                break
            else:
                route[i], route[j] = route[j], route[i]

    return tuple(route)
