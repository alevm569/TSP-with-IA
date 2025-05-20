def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a hybrid approach that combines local search with a different heuristic.

    # Use a genetic algorithm to find the best route.
    population_size = 100
    generations = 100
    mutation_rate = 0.1
    crossover_rate = 0.8

    # Create a genetic algorithm instance.
    ga = funsearch.GA(population_size, generations, mutation_rate, crossover_rate)

    # Define the fitness function.
    def fitness(route):
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm.
    best_route = ga.run(fitness, range(len(_distances)))

    # Return the best route as a tuple of city indices.
    return best_route
