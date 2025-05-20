"""Implement a new version of the find_best_route function to solve the TSP problem.
AVOID brute force solutions, create new heuristics
PROVIDE just the python code for the new version of the function, i.e. find_best_route_vx"""
import numpy as np
import math
from numpy import ndarray
import funsearch

from examples.util import read_distance_matrix



def calculate_route_distance(route: tuple[int, ...], distances: ndarray) -> float:
    """
    function to calculate the total distance of a given route.
    sum the distances between cities in the route
    """
    distance = sum(distances[route[i], route[i + 1]] for i in range(len(route) - 1))
    # add the distance from the last city to the first city
    distance += distances[route[-1], route[0]]
    return int(distance)


def evaluate(n: int) -> float:
    """
    Evaluate the find_best_route function, calculating the total distance of the best valid route.
    Penalizes invalid routes (e.g., repeated or missing cities).
    """
    matrix_distances = read_distance_matrix()

    best_route = find_best_route(matrix_distances)
    return calculate_route_distance(best_route, matrix_distances)


def find_best_route(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Objective: Find a permutation of cities that minimizes the total route distance,
    including the return to the starting city.
    
    Parameters:
    _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    You may invent or combine heuristics from scratch, or use strategies such as:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design

    Routes must include all cities exactly once and return to the starting point.
    """
    """Improved version of `find_best_route_v1`."""

    # Use a genetic algorithm to search for the best route.
    # Initialize the population of routes with random permutations of the cities.
    population = [np.random.permutation(len(_distances)) for _ in range(100)]

    # Define a fitness function to evaluate the total distance of a route.
    def fitness(route: np.ndarray) -> float:
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm until a stopping condition is met.
    while True:
        # Evaluate the fitness of each route in the population.
        fitnesses = [fitness(route) for route in population]

        # Select the best route in the population.
        best_route = population[np.argmin(fitnesses)]

        # Stop if the best route is less than or equal to 1000.
        if fitness(best_route) <= 1000:
            break

        # Create new routes by combining and mutating routes in the population.
        new_population = []
        for _ in range(100):
            parent1 = population[np.random.randint(len(population))]
            parent2 = population[np.random.randint(len(population))]

            # Combine the two parents to create a new route.
            crossover_point = np.random.randint(len(parent1))
            child = np.concatenate((parent1[:crossover_point], parent2[crossover_point:]))

            # Mutate the new route by swapping two cities.
            mutation_point1 = np.random.randint(len(child))
            mutation_point2 = np.random.randint(len(child))
            child[mutation_point1], child[mutation_point2] = child[mutation_point2], child[mutation_point1]

            new_population.append(child)

        population = new_population

