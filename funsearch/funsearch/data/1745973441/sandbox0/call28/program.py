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

    # Generate an initial solution using the nearest neighbor heuristic.
    initial_route = nearest_neighbor(_distances)

    # Apply 2-opt local search to refine the solution.
    best_route = local_search(_distances, initial_route)

    return best_route

def nearest_neighbor(_distances: np.ndarray) -> tuple[int, ...]:
    """Generates an initial route using the nearest neighbor heuristic."""
    num_cities = len(_distances)
    unvisited = set(range(num_cities))
    current_city = np.random.choice(num_cities)
    route = [current_city]

    while unvisited:
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        unvisited.remove(nearest_city)
        route.append(nearest_city)
        current_city = nearest_city

    return tuple(route)

def local_search(_distances: np.ndarray, route: tuple[int, ...]) -> tuple[int, ...]:
    """Applies 2-opt local search to refine a route."""
    best_route = route

    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            new_route = route[:i] + route[i:j][::-1] + route[j:]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(best_route, _distances):
                best_route = new_route

