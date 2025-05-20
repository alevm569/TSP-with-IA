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
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.

    Parameters:
    distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Heuristics used:
    - 2-opt
    - Tabu search

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize population of routes
    population = np.random.permutation(len(distances))

    # Perform tabu search
    best_route = tabu_search(population, distances)

    return best_route

def tabu_search(population: np.ndarray, distances: np.ndarray) -> tuple[int, ...]:
    """
    Performs tabu search to find the best route.

    Parameters:
    population (np.ndarray): Initial population of routes
    distances (np.ndarray): Square matrix of distances between cities

    Returns:
    The best route found by tabu search
    """

    # Initialize tabu list
    tabu_list = []

    # Run tabu search iterations
    for _ in range(100):
        # Select two routes from the population
        route1, route2 = np.random.choice(population, size=2, replace=False)

        # Apply 2-opt to the selected routes
        route1, route2 = two_opt(route1, route2, distances)

        # Check if the newly generated routes are valid
        if is_valid_route(route1, distances) and is_valid_route(route2, distances):
            # Add the new routes to the population
            population = np.append(population, [route1, route2], axis=0)

        # Add the best route from the current population to the tabu list
        tabu_list.append(best_route(population, distances))

    # Return the best route found by tabu search
    return best_route(population, distances)

def two_opt(route1: np.ndarray, route2: np.ndarray, distances: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Applies the 2-opt heuristic to two routes.

    Parameters:
    route1 (np.ndarray): First route
    route2 (np.ndarray): Second route
    distances (np.ndarray): Square matrix of distances between cities

    Returns:
    Two modified routes with the 2-opt operation applied
    """

    # Generate a range of indices to consider for the 2-opt operation
    indices = np.arange(len(route1))

    # Iterate over all pairs of indices
    for i in indices:
        for j in indices[i+1:]:
            # Calculate the distance difference between the original and modified routes
            distance_difference = distances[route1[i]][route2[j]] + distances[route2[i]][route1[j]] - distances[route1[i]][route1[j]] - distances[route2[j]][route2[j]]

            # If the modified route is shorter, update the routes
            if distance_difference < 0:
                route1[i], route1[j] = route1[j], route1[i]
                route2[i], route2[j] = route2[j], route2[i]

    return route1, route2

def best_route(population: np.ndarray, distances: np.ndarray) -> tuple[int, ...]:
    """
    Finds the best route from a population of routes.

    Parameters:
    population (np.ndarray): Population of routes
    distances (np.ndarray): Square matrix of distances between cities

    Returns:
    The best route in the population
    """

    # Calculate the total distance of each route
    route_distances = np.array([calculate_route_distance(route, distances) for route in population])

    # Return the route with the minimum distance
    return population[np.argmin(route_distances)]

def is_valid_route(route: np.ndarray, distances: np.ndarray) -> bool:
    """
    Checks if a route is valid, i.e., it includes all cities exactly once and returns to the starting city.

    Parameters:
    route (np.ndarray): Route to check
    distances (np.ndarray): Square matrix of distances between cities

    Returns:
    True if the route is valid, False otherwise
    """

    # Check if all cities are included in the route
    if len(set(route)) != len(distances):
        return False

    # Check if the route returns to the starting city
    if route[0] != route[-1]:
        return False

    return True

def calculate_route_distance(route: np.ndarray, distances: np.ndarray) -> float:
    """
    Calculates the total distance of a route.

    Parameters:
    route (np.ndarray): Route to calculate the distance for
    distances (np.ndarray): Square matrix of distances between cities

    Returns:
    The total distance of the route
    """

    # Calculate the distance between each pair of consecutive cities in the route
    distances_between_cities = [distances[route[i]][route[i+1]] for i in range(len(route) - 1)]

    # Add the distance between the first and last cities in the route
    distances_between_cities.append(distances[route[-1]][route[0]])

    # Return the sum of the distances between all pairs of consecutive cities
    return sum(distances_between_cities)
