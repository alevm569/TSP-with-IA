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
    Improved version of find_best_route_v2 using a hybrid heuristic.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Hybrid heuristic combining:
        - Nearest neighbor: Starting from an initial city, iteratively select the closest unvisited city.
        - Cheapest insertion: Randomly select two cities and find the cheapest insertion that does not violate the TSP constraint.

    Returns:
    tuple[int, ...]: A permutation of cities representing the best route.
    """

    n = len(distances)
    visited = np.zeros(n, dtype=bool)
    route = []

    # Start from an initial city
    current_city = np.random.randint(n)
    visited[current_city] = True
    route.append(current_city)

    # Hybrid heuristic: nearest neighbor and cheapest insertion
    for _ in range(n - 1):
        # Nearest neighbor: find the closest unvisited city
        nearest_city = np.argmin([distances[current_city][j] for j in range(n) if not visited[j]])
        visited[nearest_city] = True
        route.append(nearest_city)

        # Cheapest insertion: find the cheapest insertion that does not violate TSP
        min_distance = np.inf
        best_city = None
        for i in range(n):
            if i in route:
                continue
            distance = distances[route[-1]][i] + distances[i][current_city]
            if distance < min_distance:
                min_distance = distance
                best_city = i

        # Insert the cheapest city
        route.insert(-1, best_city)
        current_city = best_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
