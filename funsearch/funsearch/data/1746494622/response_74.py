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

    You may use at least one strategy or combine two or more heuristics from the list below:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design
        - aco (ant colony optimization)
        - genetic algorithms
        - k-opt
        - tabu search

    Routes must include all cities exactly once and return to the starting point.
    """

    # Implement your new version of the find_best_route function here.
    # You can use any of the heuristics listed in the problem description.

    # Example of using the nearest neighbor heuristic:
    # start_city = 0
    # current_city = start_city
    # route = [current_city]

    # while len(route) < len(matrix_distances):
    #     min_distance = float('inf')
    #     for city in range(len(matrix_distances)):
    #         if city not in route:
    #             distance = matrix_distances[current_city][city]
    #             if distance < min_distance:
    #                 min_distance = distance
    #                 next_city = city
    #     route.append(next_city)
    #     current_city = next_city

    # Add the return to the starting city:
    # route.append(start_city)

    # Return the route as a tuple of integers:
    return tuple(route)


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += matrix_distances[route[i]][route[(i+1) % len(route)]]
    return distance
