import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid approach."""

    # Use a genetic algorithm as the initial heuristic
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.2

    # Create a genetic algorithm object
    ga = funsearch.GA(population_size, num_generations, crossover_rate, mutation_rate)

    # Initialize the population with random routes
    population = ga.initialize_population(_distances)

    # Run the genetic algorithm
    best_route = ga.run(population, _distances)

    # Perform local search to improve the best route
    best_route = funsearch.local_search(best_route, _distances)

    return best_route
