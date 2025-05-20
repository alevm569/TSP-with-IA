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
    """Improved version of `find_best_route_v0`."""

    # Generate a random initial route
    np.random.seed(42)
    random_route = np.random.permutation(len(_distances))

    # Apply a greedy local search algorithm to find a better route
    best_route = local_search(random_route, _distances)

    return best_route

def local_search(route: np.ndarray, distances: np.ndarray) -> np.ndarray:
    """Performs a greedy local search to improve a given route."""

    best_route = route.copy()
    best_distance = calculate_route_distance(best_route, distances)

    while True:
        # Find the two closest cities in the route
        min_distance = math.inf
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                distance = distances[route[i]][route[j]]
                if distance < min_distance:
                    min_distance = distance
                    min_i = i
                    min_j = j

        # Swap the two closest cities in the route
        route[min_i], route[min_j] = route[min_j], route[min_i]

        # Check if the new route is better than the best route found so far
        new_distance = calculate_route_distance(route, distances)
        if new_distance < best_distance:
            best_route = route.copy()
            best_distance = new_distance
        else:
            # If no improvement is found, return the best route found
            return best_route

def calculate_route_distance(route: np.ndarray, distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""

    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]

