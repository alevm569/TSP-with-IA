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

    # Perform nearest neighbor search to initialize a partial route.
    current_city = 0
    partial_route = [current_city]
    remaining_cities = set(range(len(_distances)))
    remaining_cities.remove(current_city)

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        partial_route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Perform 2-opt optimization to improve the route.
    def two_opt(route):
        best_distance = calculate_route_distance(route, _distances)
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                reversed_route = route[:i] + route[i:j][::-1] + route[j:]
                distance = calculate_route_distance(reversed_route, _distances)
                if distance < best_distance:
                    best_distance = distance
                    best_route = reversed_route
        return best_route

    optimized_route = two_opt(partial_route)

