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

    You may invent or combine heuristics from scratch, or use strategies such as:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design

    Routes must include all cities exactly once and return to the starting point.
    """

    # Use the nearest neighbor heuristic to generate an initial route.
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(1, len(matrix_distances)))

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda city: matrix_distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # Close the loop by adding the starting city to the end of the route.
    route.append(start_city)

    return tuple(route)
