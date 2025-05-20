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
    """Improved version of `find_best_route_v1`."""

    # Implement a hybrid heuristic that combines two or more of the following:
    # - nearest neighbor
    # - cheapest insertion
    # - local search
    # - 2-opt
    # - hybrid or novel heuristics of your own design

    # Example hybrid heuristic:
    # 1. Use nearest neighbor to generate an initial route.
    # 2. Perform local search to improve the route.

    # Ensure that the route includes all cities exactly once and returns to the starting point.
    # Return the best route found.

    # Initialize a set of unvisited cities
    unvisited_cities = set(range(len(_distances)))

    # Start from the first city
    current_city = 0

    # Initialize the route
    route = [current_city]

    # Iterate until all cities have been visited
    while len(unvisited_cities) > 0:
        # Remove the current city from the set of unvisited cities
        unvisited_cities.remove(current_city)

        # Find the city with the shortest distance to the current city
        best_city = None
        best_distance = float('inf')
        for city in unvisited_cities:
            distance = _distances[current_city][city]
            if distance < best_distance:
                best_city = city
                best_distance = distance

        # Add the best city to the route
        route.append(best_city)

        # Set the current city to the best city
        current_city = best_city

    # Return the route
    return tuple(route)

