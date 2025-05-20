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
    Improved version of `find_best_route_v1`.

    Uses a hybrid approach combining two heuristics:
        - **Nearest neighbor:** Starts from an initial city and iteratively selects the closest unvisited city.
        - **Cheapest insertion:** Inserts the next city that minimizes the total route distance.

    Local search is also employed to refine the solution.
    """

    # Initialize route using nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    visited = set([start_city])

    while len(visited) < len(_distances):
        current_city = route[-1]
        nearest_city = np.argmin(_distances[current_city][[city for city in range(len(_distances)) if city not in visited]])
        route.append(nearest_city)
        visited.add(nearest_city)

    # Refine route using cheapest insertion heuristic
    for i in range(len(route)):
        current_city = route[i]
        cheapest_city = np.argmin(_distances[current_city][route[(i + 1) % len(route)]])
        if cheapest_city != route[(i + 1) % len(route)]:
            route[i + 1] = cheapest_city

    # Perform local search to further optimize the route
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            new_route = route[:]
            new_route[i + 1], new_route[j] = new_route[j], new_route[i + 1]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return tuple(route)

