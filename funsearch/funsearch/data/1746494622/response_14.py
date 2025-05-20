import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
np.random.seed(42)

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using genetic algorithms."""

    # Create a genetic algorithm solver
    solver = funsearch.GeneticAlgorithmSolver(
        population_size=50,
        generations=100,
        mutation_rate=0.1,
        crossover_rate=0.8,
        tournament_size=3
    )

    # Define the fitness function
    def fitness_function(route):
        return calculate_route_distance(route, _distances)

    # Run the solver to find the best route
    best_route = solver.solve(fitness_function, len(_distances))

    # Convert the best route to a tuple
    return tuple(best_route)
