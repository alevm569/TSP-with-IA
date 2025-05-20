import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set a seed for reproducibility
np.random.seed(42)

def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    This function uses a combination of the nearest neighbor and cheapest insertion heuristics,
    along with a local search strategy to find the best route.

    Args:
        _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
        tuple[int, ...]: A permutation of cities that minimizes the total route distance.
    """

    # Initialize a random starting city
    start_city = np.random.randint(len(_distances))

    # Create an empty list to store the route
    route = [start_city]

    # Create a list of unvisited cities
    unvisited_cities = list(range(len(_distances)))
    unvisited_cities.remove(start_city)

    # Use nearest neighbor to find the closest unvisited city
    while unvisited_cities:
        current_city = route[-1]
        nearest_city = unvisited_cities[np.argmin([_distances[current_city][city] for city in unvisited_cities])]
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # Use cheapest insertion to find the city that minimizes the total distance
    for _ in range(len(_distances)):
        current_city = route[-1]
        cheapest_city = unvisited_cities[np.argmin([_distances[current_city][city] for city in unvisited_cities])]
        route.append(cheapest_city)
        unvisited_cities.remove(cheapest_city)

    # Perform local search to refine the route
    for _ in range(10):
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                new_route = route[:]
                new_route[i], new_route[j] = new_route[j], new_route[i]
                if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                    route = new_route

    # Return the route as a tuple
    return tuple(route)
