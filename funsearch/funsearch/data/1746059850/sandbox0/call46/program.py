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

    # Initialize variables
    num_cities = len(_distances)
    current_city = 0
    route = [current_city]

    # Generate random starting points
    random_starts = np.random.randint(num_cities, size=num_cities)

    # Iterate until all cities have been visited
    while len(route) < num_cities:
        # Find the closest unvisited city using the nearest neighbor heuristic
        closest_city = np.argmin(_distances[current_city][:])
        if closest_city not in route:
            route.append(closest_city)

        # Swap two random cities using the 2-opt heuristic
        if np.random.rand() < 0.5:
            i, j = np.random.randint(num_cities, size=2)
            route[i], route[j] = route[j], route[i]

        # Update the current city
        current_city = route[-1]

    # Return to the starting city
    route.append(route[0])

    return tuple(route)

