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
    Improved version of the find_best_route function.

    Uses a hybrid approach combining the nearest neighbor heuristic and local search.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    A permutation of cities that minimizes the total route distance, including the return to the starting city.
    """

    # Nearest neighbor heuristic
    start_city = np.random.randint(len(matrix_distances))
    route = [start_city]
    visited = set([start_city])

    while len(visited) < len(matrix_distances):
        current_city = route[-1]
        nearest_city = np.argmin(matrix_distances[current_city][[i for i in range(len(matrix_distances)) if i not in visited]])
        route.append(nearest_city)
        visited.add(nearest_city)

    # Local search optimization
    for _ in range(100):
        i, j = np.random.randint(len(route), size=2)
        route[i], route[j] = route[j], route[i]

        if calculate_route_distance(route, matrix_distances) < calculate_route_distance(route, matrix_distances):
            pass  # Keep the new route if it has a lower distance
        else:
            route[i], route[j] = route[j], route[i]  # Restore the original route

    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of cities
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    The total distance of the route.
    """
    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return distance
