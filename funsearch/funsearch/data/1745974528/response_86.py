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


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    This function uses a hybrid approach that combines the nearest neighbor heuristic for
    initial route construction and the 2-opt local search for improvement.
    """

    # Initialize the route using the nearest neighbor heuristic
    current_city = 0
    route = [current_city]
    visited = np.zeros(len(distances), dtype=bool)
    visited[current_city] = True

    for _ in range(len(distances) - 1):
        nearest_city = np.argmin([distances[current_city][j] for j in range(len(distances)) if not visited[j]])
        route.append(nearest_city)
        visited[nearest_city] = True
        current_city = nearest_city

    # Close the route by adding the starting city
    route.append(route[0])

    # Perform local search using the 2-opt heuristic to improve the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_original = distances[route[i]][route[j]] + distances[route[(j + 1) % len(route)]][route[(i + 1) % len(route)]]
            distance_reversed = distances[route[i]][route[(j + 1) % len(route)]] + distances[route[j]][route[(i + 1) % len(route)]]

            if distance_reversed < distance_original:
                route[i:j+1] = route[j:i:-1]

    return tuple(route)
