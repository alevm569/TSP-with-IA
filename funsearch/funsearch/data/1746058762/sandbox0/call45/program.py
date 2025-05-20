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

    # Example of a hybrid heuristic:
    # 1. Use a nearest neighbor heuristic to find an initial route.
    # 2. Use a 2-opt heuristic to improve the route by swapping two consecutive edges.
    # 3. Repeat step 2 until no further improvements are found.

    # Ensure that the returned route satisfies all TSP constraints:
    # - Includes all cities exactly once.
    # - Returns to the starting city.

    # Perform local search to optimize the route
    current_route = find_best_route_v1(_distances)
    improved = True

    while improved:
        improved = False
        for i in range(len(current_route)):
            for j in range(i + 1, len(current_route)):
                # Swap two consecutive edges in the route
                new_route = current_route[:i] + current_route[j:i:-1] + current_route[j + 1:]
                if calculate_route_distance(new_route, _distances) < calculate_route_distance(current_route, _distances):
                    current_route = new_route
                    improved = True

    return current_route

