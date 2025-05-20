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

    # Perform local search using the 2-opt heuristic.
    best_route = find_initial_route(_distances)
    while True:
        improved = False
        for i in range(len(best_route)):
            for j in range(i + 1, len(best_route)):
                new_route = swap(best_route, i, j)
                if calculate_route_distance(new_route, _distances) < calculate_route_distance(best_route, _distances):
                    best_route = new_route
                    improved = True
        if not improved:
            break

    # Ensure that the route includes all cities exactly once and returns to the starting point.
    return best_route

def find_initial_route(_distances: np.ndarray) -> tuple[int, ...]:
    """Initializes a route using the nearest neighbor heuristic."""
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    return tuple(route)

def swap(route: tuple[int, ...], i: int, j: int) -> tuple[int, ...]:
    """Swaps two cities in a route."""
    return route[:i] + (route[j],) + route[i + 1:j] + (route[i],) + route[j + 1:]

def calculate_route_distance(route: tuple[int, ...], _distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    distance = 0
    for i in range(len(route)):
        distance += _distances[route[i]][route[(i + 1) % len(route)]]

