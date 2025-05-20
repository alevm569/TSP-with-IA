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

    # Use a genetic algorithm to find the best route.
    population_size = 100
    generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create the initial population.
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Run the genetic algorithm.
    for generation in range(generations):
        # Evaluate the population.
        fitness = [calculate_route_distance(route, _distances) for route in population]

        # Select the fittest routes.
        sorted_fitness = np.argsort(fitness)
        elite = population[sorted_fitness[:int(population_size * 0.2)]]

        # Create the next generation.
        offspring = []
        for i in range(population_size - len(elite)):
            parent1 = np.random.choice(elite)
            parent2 = np.random.choice(elite)

            # Crossover.
            if np.random.rand() < crossover_rate:
                crossover_point = np.random.randint(1, len(_distances) - 1)
                offspring.append(np.concatenate((parent1[:crossover_point], parent2[crossover_point:])))

            # Mutation.
            if np.random.rand() < mutation_rate:
                mutation_point1 = np.random.randint(0, len(_distances))
                mutation_point2 = np.random.randint(0, len(_distances))
                offspring.append(np.concatenate((parent1[:mutation_point1], parent2[mutation_point2], parent1[mutation_point1:mutation_point2], parent2[:mutation_point2])))

        # Add the elite routes to the next generation.
        population = elite + offspring

    # Return the best route found.
    best_fitness = np.min([calculate_route_distance(route, _distances) for route in population])
    best_route = population[np.argmin([calculate_route_distance(route, _distances) for route in population])]
    return best_route

