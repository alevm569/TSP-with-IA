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
    """
    Improved version of `find_best_route_v1` using a hybrid approach.

    The strategy combines two heuristics:
        - Nearest neighbor: Finds the closest unvisited city from the current city.
        - Cheapest insertion: Selects the city that minimizes the distance when inserted into the current route.

    The route is then iteratively refined using the 2-opt local search optimization technique.
    """

    # Initialize the route using the nearest neighbor heuristic
    current_city = 0
    route = [current_city]
    visited = set([current_city])

    # Build the route using the cheapest insertion heuristic
    for _ in range(len(_distances) - 1):
        next_city = find_cheapest_neighbor(current_city, visited, _distances)
        route.append(next_city)
        visited.add(next_city)
        current_city = next_city

    # Close the route by returning to the starting city
    route.append(route[0])

    # Refine the route using the 2-opt local search technique
    funsearch.localsearch.two_opt(route, _distances)

    return route


def find_cheapest_neighbor(city: int, visited: set[int], _distances: np.ndarray) -> int:
    """
    Finds the closest unvisited city from the given city.

    Args:
        city: The current city.
        visited: Set of already visited cities.
        _distances: The distance matrix.

    Returns:
        The ID of the closest unvisited city.
    """
    min_distance = math.inf
    cheapest_city = None

    for i in range(len(_distances)):
        if i not in visited:
            distance = _distances[city][i]
            if distance < min_distance:
                min_distance = distance
                cheapest_city = i

