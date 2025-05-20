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
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Hybrid heuristic combines nearest neighbor and 2-opt for route optimization.
    """

    # Initialize a random starting city
    current_city = np.random.randint(len(matrix_distances))
    visited = set([current_city])

    # Create an empty route
    route = [current_city]

    # Nearest neighbor heuristic
    for _ in range(len(matrix_distances) - 1):
        # Find the nearest unvisited city
        nearest_city = np.argmin([matrix_distances[current_city][j] for j in range(len(matrix_distances)) if j not in visited])
        visited.add(nearest_city)
        route.append(nearest_city)
        current_city = nearest_city

    # 2-opt heuristic
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_original = matrix_distances[route[i]][route[j]]
            distance_reversed = matrix_distances[route[i]][route[(j + 1) % len(route)]] + matrix_distances[route[(j - 1) % len(route)]][route[j]]
            if distance_reversed < distance_original:
                route[i:j+1] = route[j:i:-1]

    return tuple(route)
