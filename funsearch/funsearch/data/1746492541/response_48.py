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
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    This function uses a combination of the nearest neighbor heuristic for initialization
    and the 2-opt heuristic for route optimization.
    """

    # Initialize route using nearest neighbor heuristic
    start_city = 0
    current_city = start_city
    route = [current_city]

    # Visit each city exactly once
    unvisited_cities = set(range(len(distances)))
    unvisited_cities.remove(start_city)

    while unvisited_cities:
        # Find the nearest unvisited city
        nearest_city = min(unvisited_cities, key=lambda city: distances[current_city][city])
        unvisited_cities.remove(nearest_city)
        route.append(nearest_city)
        current_city = nearest_city

    # Close the route by returning to the starting city
    route.append(start_city)

    # Optimize the route using 2-opt heuristic
    funsearch.optimize_route(route, distances)

    return tuple(route)
