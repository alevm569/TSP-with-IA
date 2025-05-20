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
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
    - Nearest neighbor
    - Cheapest insertion

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route with the first city
    route = [0]

    # Mark the first city as visited
    visited = [False] * len(matrix_distances)
    visited[0] = True

    # Find the nearest unvisited city for each city in the route
    for _ in range(len(matrix_distances) - 1):
        current_city = route[-1]
        nearest_city = -1
        min_distance = math.inf

        for i in range(len(matrix_distances)):
            if not visited[i]:
                distance = matrix_distances[current_city][i]
                if distance < min_distance:
                    nearest_city = i
                    min_distance = distance

        # Add the nearest city to the route and mark it as visited
        route.append(nearest_city)
        visited[nearest_city] = True

    # Return to the starting city
    route.append(0)

    return tuple(route)
