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

    # Implement a hybrid heuristic combining local search and a genetic algorithm.
    # Local search can improve the quality of the initial solution provided by the genetic algorithm.

    # Initialize a genetic algorithm population.
    population = ...

    # Perform genetic algorithm iterations.
    for generation in range(max_generations):
        # Evaluate the fitness of each route in the population.
        fitness_values = [calculate_route_distance(route, _distances) for route in population]

        # Select the fittest routes for reproduction.
        best_routes = np.argsort(fitness_values)[:num_parents]

        # Create new routes through genetic operators (crossover and mutation).
        offspring = ...

        # Evaluate the fitness of the new routes.
        fitness_values = [calculate_route_distance(route, _distances) for route in offspring]

        # Update the population with the best routes.
        population = np.concatenate((population[best_routes], offspring))

    # Use local search to improve the best route found by the genetic algorithm.
    best_route = population[np.argmin(fitness_values)]
    best_route = local_search(best_route, _distances)

    return best_route

