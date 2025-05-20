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

    # Generate a random starting city
    np.random.seed(0)
    start_city = np.random.randint(len(_distances))

    # Initialize an empty route
    route = [start_city]

    # Create a set of visited cities
    visited = {start_city}

    # While all cities have not been visited
    while len(visited) < len(_distances):
        # Find the closest unvisited city to the last city in the route
        current_city = route[-1]
        min_distance = np.inf
        for i in range(len(_distances)):
            if i not in visited:
                distance = _distances[current_city][i]
                if distance < min_distance:
                    min_distance = distance
                    next_city = i

        # Add the next city to the route and mark it as visited
        route.append(next_city)
        visited.add(next_city)

    # Return to the starting city
    route.append(start_city)

    return tuple(route)

print(evaluate(0))