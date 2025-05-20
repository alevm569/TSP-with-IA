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

    This version improves upon `find_best_route_v2` by incorporating the following heuristics:

    - **Nearest neighbor heuristic:** Starting from each city, iteratively select the nearest unvisited city.
    - **Greedy insertion heuristic:** For each city in the route, find the city with the minimum distance to the current route. Insert the new city as close to the last city as possible.

    Routes must include all cities exactly once and return to the starting point.
    """

    num_cities = distances.shape[0]
    route = np.zeros(num_cities, dtype=int)

    # Initialize the route with the first city
    route[0] = 0

    # Apply nearest neighbor heuristic to generate the initial route
    for i in range(1, num_cities):
        current_city = route[i - 1]
        nearest_city = np.argmin(distances[current_city])
        while nearest_city in route:
            nearest_city = np.argmin(distances[current_city][np.isin(np.arange(num_cities), route, invert=True)])
        route[i] = nearest_city

    # Apply greedy insertion heuristic to refine the route
    for i in range(num_cities):
        current_city = route[i]
        best_distance = np.inf
        best_insertion_city = None
        for j in range(num_cities):
            if j not in route:
                distance = distances[current_city][j]
                insertion_index = np.where(route[:i] == j)[0][-1] + 1
                if distance < best_distance:
                    best_distance = distance
                    best_insertion_city = j
        route[i] = best_insertion_city

    return tuple(route)
