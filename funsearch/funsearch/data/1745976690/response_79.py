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

    Heuristic:
        - Use the nearest neighbor heuristic to initialize a route.
        - Use the cheapest insertion heuristic to add cities to the route.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize a route using the nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    visited = set([start_city])

    while len(visited) < len(distances):
        current_city = route[-1]
        next_city = np.argmin([distances[current_city][j] for j in range(len(distances)) if j not in visited])
        route.append(next_city)
        visited.add(next_city)

    # Close the route by returning to the starting city
    route.append(start_city)

    return tuple(route)
