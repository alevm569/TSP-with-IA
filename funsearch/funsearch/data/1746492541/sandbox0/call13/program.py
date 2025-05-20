"""Implement a new version of the find_best_route function to solve the TSP problem.
AVOID brute force solutions, create new heuristics and if you are going to use randomness, stabilize it by setting a seed to ensure reproducibility.
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
    return float(distance)


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

    You may use at least one strategy or combine two or more heuristics from the list below:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design
        - aco (ant colony optimization)
        - genetic algorithms
        - k-opt
        - tabu search

    Routes must include all cities exactly once and return to the starting point.
    """
    """Improved version of `find_best_route_v1` using a hybrid heuristic."""

    # Generate a random initial route
    initial_route = np.random.permutation(np.arange(len(_distances)))

    # Perform local search using the 2-opt heuristic
    best_route = local_search(_distances, initial_route)

    return best_route

def local_search(_distances: np.ndarray, initial_route: np.ndarray) -> tuple[int, ...]:
    """
    Performs local search on the given route using the 2-opt heuristic.

    Parameters:
    _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)
    initial_route (np.ndarray): The initial route to search from

    Returns:
    tuple[int, ...]: The best route found through local search
    """

    best_route = initial_route.copy()
    best_distance = calculate_route_distance(best_route, _distances)

    while True:
        for i in range(len(_distances)):
            for j in range(i + 1, len(_distances)):
                # Swap the two cities in the route
                new_route = best_route.copy()
                new_route[i], new_route[j] = new_route[j], new_route[i]

                # Calculate the distance of the new route
                new_distance = calculate_route_distance(new_route, _distances)

                # If the new route is better, update the best route
                if new_distance < best_distance:
                    best_route = new_route
                    best_distance = new_distance

        # If no further improvements are found, break out of the loop
        if best_distance == calculate_route_distance(best_route, _distances):
            break

