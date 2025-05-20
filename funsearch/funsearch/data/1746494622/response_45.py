import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

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
    Finds a permutation of cities that minimizes the total route distance.

    Args:
        matrix_distances: A square matrix of distances between cities.

    Returns:
        A tuple of integers representing the best route.
    """

    # Use a hybrid heuristic that combines several strategies
    # 1. Nearest neighbor heuristic to generate an initial route.
    # 2. Cheapest insertion heuristic to refine the route.
    # 3. Local search heuristic to optimize the route.

    # Generate an initial route using the nearest neighbor heuristic
    start_city = 0
    current_city = start_city
    route = [current_city]

    while len(route) < len(matrix_distances):
        nearest_city = np.argmin([matrix_distances[current_city][j] for j in range(len(matrix_distances)) if j not in route])
        route.append(nearest_city)
        current_city = nearest_city

    # Refine the route using the cheapest insertion heuristic
    for i in range(len(route)):
        min_distance = float('inf')
        best_city = None

        for j in range(len(matrix_distances)):
            if j not in route:
                distance = matrix_distances[route[i]][j]
                if distance < min_distance:
                    min_distance = distance
                    best_city = j

        route.insert(i + 1, best_city)

    # Optimize the route using the local search heuristic
    for _ in range(100):
        i, j = np.random.randint(0, len(route), size=2)
        route[i], route[j] = route[j], route[i]

    return tuple(route)
