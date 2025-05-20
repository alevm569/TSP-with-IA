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
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic or hybrid of heuristics here.
    # Consider using genetic algorithms, metaheuristics, or other advanced techniques.

    # Apply a 2-opt heuristic to improve the initial solution.
    best_route = find_best_route_v1(_distances)
    for i in range(len(best_route)):
        for j in range(i + 1, len(best_route)):
            new_route = best_route[:i] + (best_route[j],) + best_route[i+1:j] + (best_route[i],) + best_route[j+1:]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(best_route, _distances):
                best_route = new_route

    # Ensure that the returned route satisfies all TSP constraints:
    # - Includes all cities exactly once.
    # - Returns to the starting city.

    return best_route

