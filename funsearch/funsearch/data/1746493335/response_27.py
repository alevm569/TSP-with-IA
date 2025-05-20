import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(0)  # Set seed for reproducibility

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using genetic algorithms."""

    num_cities = len(_distances)
    population_size = 50
    generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.2

    # Create an initial population of random routes
    population = np.random.permutation(num_cities, size=(population_size, num_cities))

    # Evaluate the fitness of each route in the population
    fitness = np.array([calculate_route_distance(route, _distances) for route in population])

    # Genetic algorithm loop
    for generation in range(generations):
        # Select parents for breeding
        parents = funsearch.tournament_selection(population, fitness, population_size)

        # Create children through crossover and mutation
        children = []
        for i in range(population_size):
            parent1, parent2 = parents[np.random.randint(population_size, size=2)]
            if np.random.rand() < crossover_rate:
                child = funsearch.single_point_crossover(parent1, parent2)
            else:
                child = parent1
            if np.random.rand() < mutation_rate:
                child = funsearch.swap_mutation(child)
            children.append(child)

        # Evaluate the fitness of the new routes
        fitness_children = np.array([calculate_route_distance(route, _distances) for route in children])

        # Replace the worst routes in the population with the new routes
        population[fitness > np.max(fitness_children)] = children[fitness_children < np.min(fitness)]
        fitness = np.min(np.stack((fitness, fitness_children)), axis=0)

    # Return the best route found
    best_route_index = np.argmin(fitness)
    return population[best_route_index]
