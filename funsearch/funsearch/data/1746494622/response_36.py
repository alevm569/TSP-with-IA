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
    population_size = 50
    generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create a population of candidate routes.
    population = np.random.permutation(len(_distances)).reshape(-1, len(_distances))

    # Iterate over generations.
    for generation in range(generations):
        # Select parents.
        parents = funsearch.tournament_selection(population, _distances)

        # Create offspring.
        offspring = funsearch.crossover(parents, _distances, crossover_rate)
        funsearch.mutate(offspring, _distances, mutation_rate)

        # Evaluate offspring.
        fitness_values = np.array([calculate_route_distance(route, _distances) for route in offspring])

        # Select the best offspring.
        population = offspring[np.argsort(fitness_values)[:population_size]]

    # Return the best route.
    return population[0]
