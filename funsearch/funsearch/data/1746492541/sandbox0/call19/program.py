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
    """Improved version of `find_best_route_v2`."""

    # Initialize random number generator with a seed for reproducibility
    np.random.seed(0)

    # Use genetic algorithms to find a good route
    population_size = 100
    num_generations = 100
    mutation_rate = 0.1
    crossover_rate = 0.8

    # Create a population of random routes
    population = np.random.permutation(len(_distances)).reshape((population_size, -1))

    # Iterate over generations
    for generation in range(num_generations):
        # Evaluate the fitness of each route
        fitness = calculate_route_distance(population, _distances)

        # Select parents for reproduction
        parents = funsearch.tournament_selection(population, fitness)

        # Create new offspring through crossover and mutation
        offspring = funsearch.crossover(parents, crossover_rate)
        offspring = funsearch.mutation(offspring, mutation_rate)

        # Replace the worst routes in the population with the new offspring
        population[fitness.argsort()[:population_size - len(offspring)]] = offspring

    # Return the best route found
    best_route = population[np.argmin(fitness)]
    return best_route

