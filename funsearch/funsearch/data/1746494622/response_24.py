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

    - **Nearest Neighbor:** Starts from an initial city and iteratively selects the nearest unvisited city.
    - **Cheapest Insertion:** Chooses the city that minimizes the distance from the last visited city.

    Routes must include all cities exactly once and return to the starting point.
    """

    num_cities = len(matrix_distances)

    # Initialize the route with the first city
    route = [0]
    visited = set([0])

    # Nearest Neighbor heuristic
    for _ in range(num_cities - 1):
        current_city = route[-1]
        nearest_city = None
        min_distance = float('inf')

        # Find the nearest unvisited city
        for i in range(num_cities):
            if i not in visited:
                distance = matrix_distances[current_city][i]
                if distance < min_distance:
                    min_distance = distance
                    nearest_city = i

        # Add the nearest city to the route
        route.append(nearest_city)
        visited.add(nearest_city)

    # Return to the starting city
    route.append(0)

    return tuple(route)
