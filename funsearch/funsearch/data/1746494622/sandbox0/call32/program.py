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
    """Improved version of `find_best_route_v1` using hybrid heuristics."""

    # Initial solution using nearest neighbor heuristic
    start_city = np.random.randint(len(_distances))
    current_city = start_city
    route = [current_city]

    # Generate a set of available cities
    available_cities = set(range(len(_distances)))
    available_cities.remove(start_city)

    # Hybrid heuristic: Combine 2-opt and cheapest insertion
    while available_cities:
        # 2-opt heuristic
        best_delta = float('inf')
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                delta = 2 * _distances[route[i]][route[j]] - _distances[route[(i - 1) % len(route)]][route[i]] - _distances[route[j]][route[(j + 1) % len(route)]]
                if delta < best_delta:
                    best_delta = delta
                    best_i = i
                    best_j = j

        if best_delta < 0:
            route[best_i], route[best_j] = route[best_j], route[best_i]
        else:
            # Cheapest insertion heuristic
            best_city = None
            best_distance = float('inf')
            for city in available_cities:
                distance = _distances[current_city][city]
                if distance < best_distance:
                    best_distance = distance
                    best_city = city

            route.append(best_city)
            available_cities.remove(best_city)
            current_city = best_city

    # Ensure the route returns to the starting city
    route.append(start_city)

    return tuple(route)

