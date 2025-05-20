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
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
        - Nearest neighbor: Start from an initial city and iteratively select the nearest unvisited city.
        - 2-opt: Swap two consecutive edges in the route to potentially improve the distance.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize route using nearest neighbor
    start_city = np.random.randint(len(distances))
    route = [start_city]
    visited = set([start_city])

    while len(visited) < len(distances):
        current_city = route[-1]
        nearest_city = np.argmin([distances[current_city][j] for j in range(len(distances)) if j not in visited])
        route.append(nearest_city)
        visited.add(nearest_city)

    # Perform 2-opt optimization
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_diff = distances[route[i]][route[j]] - distances[route[(i - 1) % len(route)]][route[i]] - distances[route[j]][route[(j + 1) % len(route)]]
            if distance_diff < 0:
                route[i:j + 1] = route[j:i:-1]

    return tuple(route)
