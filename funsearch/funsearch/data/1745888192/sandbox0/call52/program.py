"""Implement a new version of the find_best_route function to solve the TSP problem.
AVOID brute force solutions, create new heuristics
PROVIDE just the python code for the new version of the function, i.e. find_best_route_vx"""
import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



def calculate_route_distance(route: tuple[int, ...], distances: ndarray) -> float:
    """
    function to calculate the total distance of a given route.
    sum the distances between cities in the route
    """
    distance = sum(distances[route[i], route[i + 1]] for i in range(len(route) - 1))
    # add the distance from the last city to the first city
    distance += distances[route[-1], route[0]]
    return int(distance)


def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.
    
    Parameters:
    _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    You may invent or combine heuristics from scratch, or use strategies such as:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design

    Routes must include all cities exactly once and return to the starting point.
    """
    """Improved version of `find_best_route_v2`."""

    # Implement a hybrid heuristic that combines multiple approaches:
    # - Nearest neighbor to generate an initial solution
    # - Cheapest insertion to refine the solution
    # - Local search to optimize the solution further

    # Example hybrid heuristic using nearest neighbor and cheapest insertion:
    n = len(_distances)
    route = list(range(n))
    np.random.shuffle(route)

    for i in range(n):
        current_city = route[i]
        nearest_city = np.argmin(_distances[current_city])
        route.insert(i + 1, nearest_city)

    for i in range(n):
        min_distance = np.inf
        min_city = None

        for j in range(n):
            if j not in route:
                distance = _distances[route[i]][j]
                if distance < min_distance:
                    min_distance = distance
                    min_city = j

        route.insert(i + 1, min_city)

    return tuple(route)

