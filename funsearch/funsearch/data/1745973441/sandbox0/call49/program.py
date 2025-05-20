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
    """Improved version of `find_best_route_v2` with hybrid heuristic."""

    # Generate a random initial route
    route = np.random.permutation(len(_distances))

    # Perform hybrid heuristic
    for _ in range(100):
        # Apply 2-opt heuristic to improve local search
        best_distance = calculate_route_distance(route, _distances)
        for i in range(len(_distances)):
            for j in range(i + 1, len(_distances)):
                new_route = route.copy()
                new_route[i:j+1] = new_route[j:i:-1]
                new_distance = calculate_route_distance(new_route, _distances)
                if new_distance < best_distance:
                    route = new_route
                    best_distance = new_distance

        # Perform local search
        for _ in range(10):
            # Randomly swap two cities in the route
            i, j = np.random.randint(0, len(_distances), 2)
            route[i], route[j] = route[j], route[i]

            # Check if the new route is better
            if calculate_route_distance(route, _distances) < calculate_route_distance(route, _distances):
                pass
            else:
                # If not, reverse the swap
                route[i], route[j] = route[j], route[i]

    return route

