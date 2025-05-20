import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using hybrid heuristics."""

    # Use a combination of local search and genetic algorithms
    population_size = 100
    num_generations = 50

    # Create a population of routes
    population = np.random.permutation(len(_distances))

    # Evaluate the fitness of each route
    fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Iterate through generations
    for generation in range(num_generations):
        # Selection
        parent1 = population[np.argmin(fitness_values)]
        parent2 = population[np.argmin(fitness_values)]

        # Crossover
        crossover_point = np.random.randint(1, len(_distances) - 1)
        child = np.concatenate((parent1[:crossover_point], parent2[crossover_point:]))

        # Mutation
        mutation_probability = 0.1
        if np.random.rand() < mutation_probability:
            mutation_index = np.random.randint(0, len(_distances))
            child[mutation_index] = np.random.permutation(np.setdiff1d(child, [mutation_index]))[0]

        # Evaluate the fitness of the new route
        fitness_value = calculate_route_distance(child, _distances)

        # Replace the least fit route with the new route
        if fitness_value < fitness_values[-1]:
            population[-1] = child
            fitness_values[-1] = fitness_value

    # Return the best route
    return population[np.argmin(fitness_values)]
