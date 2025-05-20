import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix

# Set random seed for reproducibility
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
    Find a permutation of cities that minimizes the total route distance.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
    - **Nearest neighbor:** Start from an initial city and iteratively choose the nearest unvisited city.
    - **Local search:** Repeatedly swap two random cities and check if the total distance is improved.

    Returns:
    tuple[int, ...]: A permutation of cities in the route.
    """

    # Initialize a random starting city
    start_city = np.random.randint(len(distances))

    # Create a list of unvisited cities
    unvisited_cities = list(range(len(distances)))
    unvisited_cities.remove(start_city)

    # Initialize the route with the starting city
    route = [start_city]

    # Nearest neighbor algorithm
    while unvisited_cities:
        current_city = route[-1]
        nearest_city = min(unvisited_cities, key=lambda city: distances[current_city][city])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # Local search optimization
    for _ in range(100):
        i, j = np.random.randint(len(route), size=2)
        route[i], route[j] = route[j], route[i]
        if calculate_route_distance(route, distances) < calculate_route_distance(route[:-1], distances):
            continue
        else:
            route = route[:-1]

    return tuple(route)

def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (tuple[int, ...]): A permutation of cities in the route.
    distances (np.ndarray): A square matrix of distances between cities.

    Returns:
    float: The total distance of the route.
    """

    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]

    return total_distance
