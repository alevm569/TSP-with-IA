import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
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
