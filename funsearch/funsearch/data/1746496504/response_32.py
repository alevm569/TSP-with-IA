import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set seed for reproducibility
np.random.seed(42)

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
    Improved version of find_best_route_v1.

    Uses a hybrid approach combining:
        - Nearest neighbor heuristic to generate an initial solution.
        - 2-opt local search to refine the solution.

    Returns:
        A tuple representing the best route, including the return to the starting city.
    """

    # Generate an initial solution using the nearest neighbor heuristic
    current_city = np.random.randint(len(distances))
    route = [current_city]
    visited = set([current_city])

    for _ in range(len(distances) - 1):
        next_city = np.argmin([distances[current_city][j] for j in range(len(distances)) if j not in visited])
        route.append(next_city)
        visited.add(next_city)
        current_city = next_city

    # Refine the solution using 2-opt local search
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            distance_original = distances[route[i]][route[j]] + distances[route[(j + 1) % len(route)]][route[(i + 1) % len(route)]]
            distance_reversed = distances[route[i]][route[(j + 1) % len(route)]] + distances[route[j]][route[(i + 1) % len(route)]]
            if distance_reversed < distance_original:
                route[i:j+1] = route[j:i:-1]

    return tuple(route)
