import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v1(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""
    # Use a genetic algorithm to generate candidate routes.
    population_size = 100
    num_generations = 50
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create a genetic algorithm instance.
    ga = funsearch.GeneticAlgorithm(population_size, num_generations, crossover_rate, mutation_rate)

    # Define the fitness function.
    def fitness(route: np.ndarray) -> float:
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm.
    best_route = ga.run(fitness)

    # Return the best route.
    return tuple(best_route)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v0`."""
    # Use a local search algorithm to refine the candidate routes.
    temperature = 100
    cooling_rate = 0.9
    num_iterations = 1000

    # Create a local search instance.
    ls = funsearch.LocalSearch(temperature, cooling_rate, num_iterations)

    # Run the local search.
    best_route = ls.run(_distances)

    # Return the best route.
    return tuple(best_route)


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
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
