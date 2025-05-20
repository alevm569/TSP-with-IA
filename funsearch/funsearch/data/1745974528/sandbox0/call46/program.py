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
    """Improved version of `find_best_route_v2` using genetic algorithms."""
    num_cities = len(_distances)
    population_size = 100
    generations = 100

    # Initialize population
    population = np.random.permutation(num_cities, size=(population_size, num_cities))

    # Fitness function
    def fitness(route: tuple[int, ...]) -> float:
        distance = 0
        for i in range(num_cities):
            distance += _distances[route[i]][route[(i + 1) % num_cities]]
        return distance

    # Genetic operators
    def crossover(parent1: tuple[int, ...], parent2: tuple[int, ...]) -> tuple[int, ...]:
        mask = np.random.randint(2, size=num_cities)
        child = np.where(mask == 0, parent1, parent2)
        return child

    def mutation(route: tuple[int, ...]) -> tuple[int, ...]:
        i, j = np.random.randint(num_cities, size=2)
        route[i], route[j] = route[j], route[i]
        return route

    # Genetic algorithm
    for generation in range(generations):
        # Evaluate population
        fitness_values = np.array([fitness(route) for route in population])

        # Select parents
        parents = population[np.argsort(fitness_values)[:population_size // 2]]

        # Create new population
        population = []
        for _ in range(population_size):
            parent1, parent2 = np.random.choice(parents, size=2)
            child = crossover(parent1, parent2)
            child = mutation(child)
            population.append(child)

    # Return best route
    best_route = population[np.argmin([fitness(route) for route in population])]

