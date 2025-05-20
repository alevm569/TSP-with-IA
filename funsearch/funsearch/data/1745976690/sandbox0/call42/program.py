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
    """Improved version of `find_best_route_v2` using a hybrid approach."""

    # Create a list of candidate routes using the nearest neighbor heuristic
    routes = [nearest_neighbor(_distances)]

    # Perform local search on each candidate route to find the best permutation
    best_route = local_search(routes[0], _distances)

    return best_route


def nearest_neighbor(_distances: np.ndarray) -> tuple[int, ...]:
    """Finds a route using the nearest neighbor heuristic."""
    num_cities = len(_distances)
    route = []
    unvisited = set(range(num_cities))

    # Start from the first city
    current_city = 0
    unvisited.remove(current_city)

    # Visit each city in order of their distance from the current city
    while unvisited:
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)
        current_city = nearest_city

    # Return to the starting city
    route.append(0)

    return tuple(route)


def local_search(route: tuple[int, ...], _distances: np.ndarray) -> tuple[int, ...]:
    """Performs local search on a candidate route."""

    # Generate all possible 2-opt swaps
    swaps = [(i, j) for i in range(len(route)) for j in range(i + 1, len(route))]

    # Evaluate each swap and select the best one
    best_route = route
    best_distance = calculate_route_distance(route, _distances)

    for i, j in swaps:
        new_route = route[:i] + route[j:i:-1] + route[j + 1:]
        new_distance = calculate_route_distance(new_route, _distances)

        if new_distance < best_distance:
            best_route = new_route
            best_distance = new_distance

