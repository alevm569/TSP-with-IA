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

    # Apply a two-opt heuristic to improve the initial solution.
    best_route = find_best_route_v1(_distances)
    improved_route = two_opt(best_route, _distances)

    # If the two-opt heuristic improves the solution, return the improved route.
    if calculate_route_distance(improved_route, _distances) < calculate_route_distance(best_route, _distances):
        return improved_route

    # Otherwise, return the original best route.
    return best_route


def two_opt(route: tuple[int, ...], distances: np.ndarray) -> tuple[int, ...]:
    """
    Two-opt heuristic for improving a TSP route.

    Parameters:
    route (tuple[int, ...]): The current route.
    distances (np.ndarray): A square matrix of distances between cities.

    Returns:
    tuple[int, ...]: The improved route.
    """

    # Iterate over all pairs of cities in the route.
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            # Create a new route by reversing the segment between cities i and j.
            new_route = route[:i] + route[i:j+1][::-1] + route[j+1:]

            # Calculate the distance of the new route.
            new_distance = calculate_route_distance(new_route, distances)

            # If the new route is shorter, return it.
            if new_distance < calculate_route_distance(route, distances):
                return new_route

    # No improvement was found, return the original route.

