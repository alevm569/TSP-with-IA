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

    # Create a set to track visited cities.
    visited = set()

    # Start from the first city.
    current_city = 0

    # Create a list to store the route.
    route = []

    # Iterate until all cities have been visited.
    while len(visited) < len(_distances):
        # Add the current city to the route.
        route.append(current_city)

        # Mark the current city as visited.
        visited.add(current_city)

        # Find the next city to visit.
        best_distance = math.inf
        for next_city in range(len(_distances)):
            if next_city not in visited:
                distance = _distances[current_city][next_city]
                if distance < best_distance:
                    best_distance = distance
                    next_city = next_city

        # Set the current city to the next city.
        current_city = next_city

    # Return to the starting city.
    route.append(route[0])

    return tuple(route)

