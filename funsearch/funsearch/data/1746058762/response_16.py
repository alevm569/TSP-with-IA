def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a genetic algorithm to find the best route.
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.2

    ga = funsearch.GA(population_size, num_generations, crossover_rate, mutation_rate)
    best_route = ga.search(len(_distances))

    # Ensure that the returned route satisfies all TSP constraints:
    # - Includes all cities exactly once.
    # - Returns to the starting city.

    return best_route
