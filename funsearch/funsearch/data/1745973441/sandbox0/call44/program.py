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
    """Improved version of `find_best_route_v2` using genetic algorithms."""

    # Initialize population of routes
    population = [np.random.permutation(len(_distances)) for _ in range(100)]

    # Define fitness function
    def fitness(route: np.ndarray) -> float:
        return calculate_route_distance(route, _distances)

    # Run genetic algorithm
    best_route = funsearch.genetic_algorithm(population, fitness, max_generations=1000)

    return best_route


def find_best_route_v4(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v3` using hybrid approach."""

    # Combine genetic algorithm with local search
    best_route = find_best_route(_distances)
    best_distance = fitness(best_route)

    while True:
        # Perform local search on best route
        improved_route = local_search(best_route, _distances)
        improved_distance = fitness(improved_route)

        if improved_distance < best_distance:
            best_route = improved_route
            best_distance = improved_distance
        else:
            break

