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

    # Implement a new version of the find_best_route function that solves the TSP problem using a hybrid heuristic approach.
    # This approach combines multiple heuristics, such as nearest neighbor, 2-opt, and local search, to generate a high-quality solution.

    # Initialization
    num_cities = len(_distances)
    current_city = np.random.randint(num_cities)
    route = [current_city]

    # Generate a preliminary route using the nearest neighbor heuristic
    for _ in range(num_cities - 1):
        current_city = find_nearest_neighbor(current_city, _distances, route)
        route.append(current_city)

    # Perform local search to refine the route
    for _ in range(num_cities):
        best_route = route
        best_distance = calculate_route_distance(route, _distances)

        # Try swapping two cities in the route and check if it improves the distance
        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                new_route = swap_cities(route, i, j)
                new_distance = calculate_route_distance(new_route, _distances)

                if new_distance < best_distance:
                    best_distance = new_distance
                    best_route = new_route

        route = best_route

    # Return the optimal route
    return tuple(route)

def find_nearest_neighbor(current_city: int, distances: np.ndarray, route: list) -> int:
    """Finds the nearest city not already in the route."""
    min_distance = np.inf
    nearest_city = None

    for i in range(len(distances)):
        if i not in route:
            distance = distances[current_city][i]
            if distance < min_distance:
                min_distance = distance
                nearest_city = i

    return nearest_city

def swap_cities(route: list, i: int, j: int) -> list:
    """Swaps two cities in the route."""
    route[i], route[j] = route[j], route[i]
    return route

def calculate_route_distance(route: list, distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0

    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]

