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

    This function implements a hybrid heuristic combining the nearest neighbor and cheapest insertion heuristics.
    """

    # Initialization
    num_cities = len(matrix_distances)
    unvisited_cities = set(range(num_cities))
    route = []

    # Start from an arbitrary city
    current_city = np.random.choice(list(unvisited_cities))
    unvisited_cities.remove(current_city)

    # Build the route iteratively
    while unvisited_cities:
        # Find the nearest unvisited city
        nearest_city = min(unvisited_cities, key=lambda city: matrix_distances[current_city][city])

        # Find the cheapest insertion point
        cheapest_insertion_index = -1
        min_distance = float('inf')
        for i in range(len(route)):
            distance = matrix_distances[route[i]][nearest_city]
            if distance < min_distance:
                cheapest_insertion_index = i
                min_distance = distance

        # Insert the nearest city at the cheapest insertion point
        if cheapest_insertion_index == -1:
            route.append(nearest_city)
        else:
            route.insert(cheapest_insertion_index + 1, nearest_city)

        # Remove the visited city
        unvisited_cities.remove(nearest_city)
        current_city = nearest_city

    # Close the route by adding the starting city
    route.append(route[0])

    return tuple(route)
