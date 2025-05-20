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


def find_best_route(matrix_distances: np.ndarray) -> tuple[int, ...]:
    """
    Finds the best route using a combination of nearest neighbor and 2-opt heuristics.

    Args:
        matrix_distances: A square matrix of distances between cities.

    Returns:
        A tuple representing the best route.
    """

    # Initialize a random starting city.
    np.random.seed(42)
    start_city = np.random.randint(len(matrix_distances))

    # Use nearest neighbor to find an initial route.
    route = nearest_neighbor(matrix_distances, start_city)

    # Apply 2-opt to improve the route.
    route = two_opt(matrix_distances, route)

    return route


def nearest_neighbor(matrix_distances: np.ndarray, start_city: int) -> tuple[int, ...]:
    """
    Finds a route using the nearest neighbor heuristic.

    Args:
        matrix_distances: A square matrix of distances between cities.
        start_city: The starting city.

    Returns:
        A tuple representing the route.
    """

    # Initialize the route with the starting city.
    route = [start_city]

    # Create a list of unvisited cities.
    unvisited_cities = list(range(len(matrix_distances)))
    unvisited_cities.remove(start_city)

    # Find the nearest city to the current city and add it to the route.
    while unvisited_cities:
        current_city = route[-1]
        nearest_city = unvisited_cities[np.argmin(matrix_distances[current_city][unvisited_cities])]
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    return tuple(route)


def two_opt(matrix_distances: np.ndarray, route: tuple[int, ...]) -> tuple[int, ...]:
    """
    Improves a route using the 2-opt heuristic.

    Args:
        matrix_distances: A square matrix of distances between cities.
        route: The route to improve.

    Returns:
        A tuple representing the improved route.
    """

    # Generate all possible 2-opt swaps.
    swaps = [(i, j) for i in range(len(route)) for j in range(i + 1, len(route))]

    # Find the best swap that improves the route distance.
    best_distance = calculate_route_distance(route, matrix_distances)
    best_swap = None

    for i, j in swaps:
        new_route = route[:i] + route[j:i:-1] + route[j+1:]
        new_distance = calculate_route_distance(new_route, matrix_distances)

        if new_distance < best_distance:
            best_distance = new_distance
            best_swap = (i, j)

    # If a better swap was found, return the improved route.
    if best_swap:
        return route[:best_swap[0]] + route[best_swap[1]:best_swap[0):-1] + route[best_swap[1]+1:]

    # Otherwise, return the original route.
    return route


def calculate_route_distance(route: tuple[int, ...], matrix_distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Args:
        route: The route to calculate the distance for.
        matrix_distances: A square matrix of distances between cities.

    Returns:
        The total distance of the route.
    """

    # Calculate the distance between each pair of consecutive cities in the route.
    distances = [matrix_distances[route[i]][route[i+1]] for i in range(len(route) - 1)]

    # Add the distance between the last and first city.
    distances.append(matrix_distances[route[-1]][route[0]])

    # Return the sum of the distances.
    return np.sum(distances)
