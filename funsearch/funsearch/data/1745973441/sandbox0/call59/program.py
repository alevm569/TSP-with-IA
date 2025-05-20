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
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Generate a random initial route
    num_cities = len(_distances)
    initial_route = np.random.permutation(num_cities)

    # Use a local search algorithm to refine the route
    def local_search(route):
        best_route = route
        best_distance = calculate_route_distance(route, _distances)

        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                # Swap two cities in the route
                new_route = route.copy()
                new_route[i], new_route[j] = new_route[j], new_route[i]

                # Calculate the distance of the new route
                new_distance = calculate_route_distance(new_route, _distances)

                # If the new route is better, update the best route
                if new_distance < best_distance:
                    best_route = new_route
                    best_distance = new_distance

        return best_route, best_distance

    # Run the local search algorithm until convergence
    best_route, best_distance = local_search(initial_route)
    while True:
        new_route, new_distance = local_search(best_route)
        if new_distance == best_distance:
            break
        else:
            best_route, best_distance = new_route, new_distance

