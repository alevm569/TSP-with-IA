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
    """Improved version of `find_best_route_v1` using 2-opt heuristic."""

    # Initialize a random route
    route = np.random.permutation(len(_distances))

    # Iterate until convergence
    while True:
        # Apply 2-opt heuristic to find two edges to swap
        best_delta = float('inf')
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                delta = _distances[route[i]][route[(j-1)%len(route)]] + _distances[route[(j+1)%len(route)]][route[i]] - _distances[route[i]][route[j]] - _distances[route[(j-1)%len(route)]][route[(j+1)%len(route)]]
                if delta < best_delta:
                    best_delta = delta
                    best_i, best_j = i, j

        # If no improvement is found, return the current route
        if best_delta == float('inf'):
            return route

        # Swap the two edges and update the route
        route[best_i], route[best_j] = route[best_j], route[best_i]

