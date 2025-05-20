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
    # Use a hybrid approach combining nearest neighbor and 2-opt heuristics
    def hybrid_heuristic(distances):
        # Nearest neighbor heuristic
        start = 0
        route = [start]
        unvisited = set(range(1, len(distances)))

        while unvisited:
            current = route[-1]
            nearest = min(unvisited, key=lambda city: distances[current][city])
            route.append(nearest)
            unvisited.remove(nearest)

        # 2-opt heuristic to improve route
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                distance_difference = distances[route[i]][route[j]] + distances[route[i + 1]][route[j - 1]] - distances[route[i]][route[i + 1]] - distances[route[j]][route[j - 1]]
                if distance_difference < 0:
                    route[i + 1:j] = reversed(route[i + 1:j])

        return tuple(route)

    # Use the hybrid heuristic to find the best route
    best_route = hybrid_heuristic(_distances)

