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

    # Implement a hybrid approach that combines multiple heuristics, such as:

    # 1. Nearest neighbor heuristic to generate an initial candidate route.
    # 2. Cheapest insertion heuristic to refine the candidate route.
    # 3. Local search algorithm to optimize the route by iteratively swapping two cities.

    # Perform multiple iterations of the hybrid approach and return the route with the lowest total distance.
    best_route = None
    best_distance = float('inf')

    for _ in range(10):  # Adjust the number of iterations as needed
        # Run the hybrid approach to generate a candidate route
        candidate_route = hybrid_heuristic(_distances)

        # Calculate the distance of the candidate route
        candidate_distance = calculate_route_distance(candidate_route, _distances)

        # Update the best route if necessary
        if candidate_distance < best_distance:
            best_distance = candidate_distance
            best_route = candidate_route

    return best_route


def hybrid_heuristic(_distances: np.ndarray) -> tuple[int, ...]:
    # Implement the hybrid heuristic that combines multiple heuristics.

