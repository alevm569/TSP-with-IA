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

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2`.

    Uses a hybrid approach combining local search with 2-opt optimization.
    """
    # Initialize a random route
    route = np.random.permutation(np.arange(len(matrix_distances)))

    # Local search
    for _ in range(100):
        # Choose two random cities
        i, j = np.random.randint(len(route), size=2)
        # Swap the two cities
        route[i], route[j] = route[j], route[i]

    # 2-opt optimization
    for _ in range(100):
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                # Calculate the distance of the current route
                distance = calculate_route_distance(route, matrix_distances)

                # Swap the two cities
                route[i], route[j] = route[j], route[i]

                # Calculate the distance of the modified route
                new_distance = calculate_route_distance(route, matrix_distances)

                # If the new distance is smaller, keep the modified route
                if new_distance < distance:
                    distance = new_distance
                else:
                    # Otherwise, swap the two cities back
                    route[i], route[j] = route[j], route[i]

    return route


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += matrix_distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
