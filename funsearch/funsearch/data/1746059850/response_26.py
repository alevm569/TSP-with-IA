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

    best_route = find_best_route_v3(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route
    population_size = 100
    generations = 50
    mutation_rate = 0.1

    # Create a genetic algorithm object
    ga = funsearch.GeneticAlgorithm(population_size, generations, mutation_rate)

    # Create a tournament selection object
    tournament_selection = funsearch.TournamentSelection()

    # Create a crossover operator
    crossover_operator = funsearch.SinglePointCrossover()

    # Create a mutation operator
    mutation_operator = funsearch.SwapMutation()

    # Create a fitness function
    fitness_function = funsearch.FitnessFunction(evaluate)

    # Create a population of routes
    population = ga.create_population(len(_distances))

    # Run the genetic algorithm
    best_route = ga.run(population, tournament_selection, crossover_operator, mutation_operator, fitness_function)

    # Return the best route
    return best_route
