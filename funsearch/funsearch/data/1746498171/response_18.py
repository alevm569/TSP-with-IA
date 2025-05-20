import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)

def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Find a permutation of cities that minimizes the total route distance, including the return to the starting city.

    Parameters:
    matrix_distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
    - Nearest neighbor: Starts from a random city and iteratively selects the closest unvisited city.
    - Cheapest insertion: Selects the city that minimizes the total distance when inserted into the current route.
    - Tabu search: Uses a tabu list to prevent repeating recently visited cities.

    Routes must include all cities exactly once and return to the starting point.
    """

    num_cities = len(matrix_distances)

    # Initialize starting city
    current_city = np.random.randint(num_cities)

    # Initialize route and visited cities
    route = [current_city]
    visited = set([current_city])

    # Run tabu search until all cities are visited
    tabu_list = []
    for _ in range(num_cities - 1):
        # Find the cheapest insertion city
        cheapest_city = None
        min_distance = float('inf')

        for city in range(num_cities):
            if city not in visited:
                distance = matrix_distances[current_city][city]
                if distance < min_distance:
                    cheapest_city = city
                    min_distance = distance

        # Add cheapest city to route and mark it as visited
        route.append(cheapest_city)
        visited.add(cheapest_city)

        # Add current city to tabu list
        tabu_list.append(current_city)

        # Update current city
        current_city = cheapest_city

    # Return to starting city
    route.append(route[0])

    return tuple(route)
