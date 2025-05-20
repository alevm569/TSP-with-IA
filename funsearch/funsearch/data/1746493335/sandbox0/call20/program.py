"""Implement a new version of the find_best_route function to solve the TSP problem.
AVOID brute force solutions, create new heuristics and if you are going to use randomness, stabilize it by setting a seed to ensure reproducibility.
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
    return float(distance)


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

    You may use at least one strategy or combine two or more heuristics from the list below:
        - nearest neighbor
        - cheapest insertion
        - local search
        - 2-opt
        - hybrid or novel heuristics of your own design
        - aco (ant colony optimization)
        - genetic algorithms
        - k-opt
        - tabu search

    Routes must include all cities exactly once and return to the starting point.
    """
    """Improved version of `find_best_route_v1` using genetic algorithms."""

    num_cities = len(_distances)
    population_size = 50
    generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.2

    # Create an initial population of random routes
    population = np.random.permutation(num_cities, size=(population_size, num_cities))

    # Evaluate the fitness of each route in the population
    fitness = np.array([calculate_route_distance(route, _distances) for route in population])

    # Genetic algorithm loop
    for generation in range(generations):
        # Select parents for breeding
        parents = funsearch.tournament_selection(population, fitness, population_size)

        # Create children through crossover and mutation
        children = []
        for i in range(population_size):
            parent1, parent2 = parents[np.random.randint(population_size, size=2)]
            if np.random.rand() < crossover_rate:
                child = funsearch.single_point_crossover(parent1, parent2)
            else:
                child = parent1
            if np.random.rand() < mutation_rate:
                child = funsearch.swap_mutation(child)
            children.append(child)

        # Evaluate the fitness of the new routes
        fitness_children = np.array([calculate_route_distance(route, _distances) for route in children])

        # Replace the worst routes in the population with the new routes
        population[fitness > np.max(fitness_children)] = children[fitness_children < np.min(fitness)]
        fitness = np.min(np.stack((fitness, fitness_children)), axis=0)

    # Return the best route found
    best_route_index = np.argmin(fitness)
    return population[best_route_index]

