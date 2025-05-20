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

    Heuristic: Use a hybrid heuristic that combines the nearest neighbor and cheapest insertion heuristics.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize the route with the starting city
    route = [0]

    # Use the nearest neighbor heuristic to find the next city
    while len(route) < len(distances):
        current_city = route[-1]
        next_city = np.argmin(distances[current_city])
        if next_city not in route:
            route.append(next_city)

    # Use the cheapest insertion heuristic to refine the route
    for i in range(len(route)):
        best_distance = float('inf')
        best_city = None
        for j in range(len(distances)):
            if j not in route:
                distance = distances[route[i]][j]
                if distance < best_distance:
                    best_distance = distance
                    best_city = j
        route.insert(i + 1, best_city)

    return tuple(route)
