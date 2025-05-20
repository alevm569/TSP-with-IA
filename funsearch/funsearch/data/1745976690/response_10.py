import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route
    population_size = 100
    generations = 100
    mutation_rate = 0.05
    crossover_rate = 0.8

    # Create a genetic algorithm object
    ga = funsearch.GeneticAlgorithm(
        population_size=population_size,
        generations=generations,
        mutation_rate=mutation_rate,
        crossover_rate=crossover_rate,
        fitness_function=calculate_route_distance,
        population_initializer=funsearch.RandomPermutationInitializer(),
        crossover_operator=funsearch.OrderCrossover(),
        mutation_operator=funsearch.ReverseMutation(),
        selection_operator=funsearch.RouletteWheelSelection(),
    )

    # Run the genetic algorithm
    best_route = ga.run(_distances)

    # Return the best route
    return best_route
