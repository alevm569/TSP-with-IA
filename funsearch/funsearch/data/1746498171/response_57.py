import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

np.random.seed(42)  # Set seed for reproducibility

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route(distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2`.

    Uses a hybrid approach combining the nearest neighbor heuristic, 2-opt local search,
    and a pheromone-based ant colony optimization algorithm.
    """

    # Nearest neighbor heuristic
    start_city = np.random.randint(len(distances))
    route = [start_city]
    visited = set([start_city])

    for _ in range(len(distances) - 1):
        current_city = route[-1]
        nearest_city = np.argmin([distance for i, distance in enumerate(distances[current_city]) if i not in visited])
        route.append(nearest_city)
        visited.add(nearest_city)

    # 2-opt local search
    improved = True
    while improved:
        improved = False
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                distance_original = distances[route[i]][route[j]]
                distance_reversed = distances[route[i]][route[(j - 1)]] + distances[route[j]][route[(i + 1)]]
                if distance_reversed < distance_original:
                    route[i+1:j] = route[j-1:i:-1]
                    improved = True

    # Ant colony optimization
    aco = funsearch.ACO(distances)
    best_route = aco.solve()

    return best_route
