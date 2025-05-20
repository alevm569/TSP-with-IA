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
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    Hybrid heuristic combines:
        - Nearest neighbor for initial route construction.
        - 2-opt for local search optimization.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: A permutation of cities representing the best route.
    """

    # Initialize route using nearest neighbor heuristic
    start_city = np.random.randint(len(distances))
    route = [start_city]
    unvisited = set(range(len(distances))) - {start_city}

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Perform local search using 2-opt heuristic
    for _ in range(100):  # Number of iterations
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                distance_difference = distances[route[i]][route[j]] - distances[route[(i + 1) % len(route)]][route[(j - 1) % len(route)]]
                if distance_difference < 0:
                    route[i+1:j] = route[j-1:i:-1]

    return tuple(route)
