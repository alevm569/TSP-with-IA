import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 using a hybrid heuristic.

    Hybrid heuristic combines two or more of the following:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - tabu search

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize an empty route
    route = []

    # Use the nearest neighbor heuristic to generate an initial route
    current_city = np.random.randint(len(_distances))
    for _ in range(len(_distances)):
        route.append(current_city)
        next_city = np.argmin(_distances[current_city])
        current_city = next_city

    # Perform tabu search to improve the route
    tabu_list = []
    for i in range(100):
        # Choose two random cities to swap
        city1, city2 = np.random.randint(len(_distances), size=2)

        # Check if the swap is valid and not in the tabu list
        if city1 != city2 and city1 not in tabu_list and city2 not in tabu_list:
            route[city1], route[city2] = route[city2], route[city1]

            # Add the swap to the tabu list
            tabu_list.append(city1)
            tabu_list.append(city2)

            # Check if the new route is better than the current best route
            if calculate_route_distance(route, _distances) < calculate_route_distance(best_route, _distances):
                best_route = route

    return best_route
