import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route_v1(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""

    # Generate a random initial route
    np.random.seed(42)
    random_route = np.random.permutation(len(_distances))

    # Apply a greedy local search algorithm to find a better route
    best_route = local_search(random_route, _distances)

    return best_route

def local_search(route: np.ndarray, distances: np.ndarray) -> np.ndarray:
    """Performs a greedy local search to improve a given route."""

    best_route = route.copy()
    best_distance = calculate_route_distance(best_route, distances)

    while True:
        # Find the two closest cities in the route
        min_distance = math.inf
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                distance = distances[route[i]][route[j]]
                if distance < min_distance:
                    min_distance = distance
                    min_i = i
                    min_j = j

        # Swap the two closest cities in the route
        route[min_i], route[min_j] = route[min_j], route[min_i]

        # Check if the new route is better than the best route found so far
        new_distance = calculate_route_distance(route, distances)
        if new_distance < best_distance:
            best_route = route.copy()
            best_distance = new_distance
        else:
            # If no improvement is found, return the best route found
            return best_route

def calculate_route_distance(route: np.ndarray, distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""

    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]

    return total_distance
