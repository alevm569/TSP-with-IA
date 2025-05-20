import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithms."""

    # Define the fitness function
    def fitness(route: tuple[int, ...]) -> float:
        return -calculate_route_distance(route, _distances)

    # Define the genetic operator
    crossover = funsearch.crossover.uniform_crossover()
    mutation = funsearch.mutation.inversion_mutation()

    # Initialize the population
    population = funsearch.population.generational_population(
        fitness=fitness,
        population_size=50,
        chromosome_length=len(_distances),
        crossover_operator=crossover,
        mutation_operator=mutation,
    )

    # Run the genetic algorithm
    population = funsearch.algorithms.genetic_algorithm(population, num_generations=100)

    # Return the best route found
    return population.best_chromosome.solution
