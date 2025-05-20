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

    # Use a combination of local search and the nearest neighbor heuristic.
    # Local search will refine the route, while the nearest neighbor heuristic will guide the search.

    # Initialize the current route.
    current_route = list(range(len(_distances)))
    np.random.shuffle(current_route)

    # Run local search.
    for i in range(100):  # Number of iterations
        # Find the best neighbor route.
        best_neighbor = None
        best_distance = float('inf')

        for j in range(len(current_route)):
            for k in range(j + 1, len(current_route)):
                # Swap two cities in the route.
                new_route = current_route[:]
                new_route[j], new_route[k] = new_route[k], new_route[j]

                # Calculate the distance of the new route.
                distance = calculate_route_distance(new_route, _distances)

                # Update the best neighbor route.
                if distance < best_distance:
                    best_neighbor = new_route
                    best_distance = distance

        # If the best neighbor route is better, update the current route.
        if best_distance < calculate_route_distance(current_route, _distances):
            current_route = best_neighbor

    # Return the best route as a tuple of city indices.
    return tuple(current_route)

print(evaluate(0))
