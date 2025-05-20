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
    # Use a combination of heuristics and algorithms to find the best route.
    # Use a genetic algorithm to generate candidate routes.
    population_size = 100
    num_generations = 50
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create a genetic algorithm instance.
    ga = funsearch.GeneticAlgorithm(population_size, num_generations, crossover_rate, mutation_rate)

    # Use a local search algorithm to refine the candidate routes.
    temperature = 100
    cooling_rate = 0.9
    num_iterations = 1000

    # Create a local search instance.
    ls = funsearch.LocalSearch(temperature, cooling_rate, num_iterations)

    # Run the genetic algorithm.
    candidate_routes = ga.run(fitness)

    # Run the local search on the candidate routes.
    best_route = ls.run(_distances, candidate_routes)

    # Return the best route.
    return tuple(best_route)

