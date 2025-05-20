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


def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using ACO."""
    np.random.seed(42)  # Set a seed for reproducibility

    # Initialize ACO parameters
    num_ants = 10
    num_iterations = 100
    alpha = 1  # pheromone importance
    beta = 2  # distance importance
    rho = 0.1  # pheromone evaporation rate

    # Create an ACO optimizer
    optimizer = funsearch.ACO(
        fitness_function=calculate_route_distance,
        distance_matrix=_distances,
        num_ants=num_ants,
        num_iterations=num_iterations,
        alpha=alpha,
        beta=beta,
        rho=rho,
    )

    # Run the optimization
    best_route = optimizer.optimize()

    return best_route
