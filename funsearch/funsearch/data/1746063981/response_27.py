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

    Heuristics used:
        - Nearest neighbor: Start from the first city and iteratively select the city with the minimum distance to the current city.
        - Cheapest insertion: Find the city that, when added to the route, results in the lowest total distance.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize variables
    num_cities = len(distances)
    route = [0]
    visited = [False] * num_cities

    # Nearest neighbor heuristic
    current_city = 0
    for _ in range(num_cities - 1):
        visited[current_city] = True
        next_city = -1
        min_distance = math.inf
        for i in range(num_cities):
            if not visited[i] and distances[current_city][i] < min_distance:
                next_city = i
                min_distance = distances[current_city][i]
        route.append(next_city)
        current_city = next_city

    # Return to the starting city
    route.append(0)

    return tuple(route)
