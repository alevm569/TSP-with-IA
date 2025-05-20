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
    """Improved version of `find_best_route_v1` using a hybrid heuristic."""

    # Initialize population of routes
    population = np.random.permutation(len(_distances))

    # Run iterated local search to improve routes
    for _ in range(100):
        # Select two routes from population
        route1, route2 = np.random.choice(population, size=2, replace=False)

        # Perform 2-opt move between the two routes
        best_distance = math.inf
        for i in range(len(route1)):
            for j in range(len(route2)):
                new_distance = calculate_route_distance(route1[:i] + route2[j:] + route1[i:], _distances)
                if new_distance < best_distance:
                    best_distance = new_distance
                    best_route = route1[:i] + route2[j:] + route1[i:]

        # Replace worst route in population with the improved route
        population[np.argmax([calculate_route_distance(route, _distances) for route in population])] = best_route

    # Return the best route found
    return population[np.argmin([calculate_route_distance(route, _distances) for route in population])]


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]

