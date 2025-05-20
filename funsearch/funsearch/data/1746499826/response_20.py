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

    best_route = find_best_route_v2(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Seed the random number generator for reproducibility
    np.random.seed(42)

    # Implement a hybrid heuristic that combines the nearest neighbor and cheapest insertion algorithms
    def hybrid_heuristic(distances):
        # Start with a random city as the initial city
        current_city = np.random.randint(len(distances))
        route = [current_city]

        # Visit each city once
        remaining_cities = set(range(len(distances))) - {current_city}

        while remaining_cities:
            # Find the nearest unvisited city
            nearest_city = min(remaining_cities, key=lambda c: distances[current_city][c])
            route.append(nearest_city)
            remaining_cities.remove(nearest_city)

            # Find the cheapest city to insert into the route
            cheapest_city = min(route[:-1], key=lambda c: distances[route[-1]][c])
            route.insert(route.index(nearest_city), cheapest_city)

        # Return to the starting city
        route.append(route[0])

        return tuple(route)

    # Use the hybrid heuristic to find the best route
    best_route = hybrid_heuristic(_distances)

    return best_route
