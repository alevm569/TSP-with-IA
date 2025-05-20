import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set a seed for reproducibility

def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with ACO."""

    # ACO parameters
    num_ants = 10
    num_iterations = 100
    pheromone_decay = 0.9
    alpha = 1
    beta = 2

    # Initialize pheromone matrix
    pheromones = np.ones_like(_distances) / len(_distances)

    # Generate initial routes
    routes = np.random.permutation(len(_distances)).reshape(-1, len(_distances))

    # ACO loop
    for _ in range(num_iterations):
        # Update pheromones
        for route in routes:
            for i in range(len(route)):
                pheromones[route[i]][route[(i + 1) % len(route)]] += 1 / calculate_route_distance(route, _distances)

        # Update routes
        routes = funsearch.aco.ant_colony_optimization(
            _distances,
            num_ants=num_ants,
            pheromones=pheromones,
            alpha=alpha,
            beta=beta,
        )

    # Return the best route
    best_route = routes[np.argmin([calculate_route_distance(route, _distances) for route in routes])]
    return best_route
