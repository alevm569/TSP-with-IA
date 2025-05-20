def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithms."""

    # Define the population size and number of generations
    population_size = 100
    num_generations = 50

    # Create a genetic algorithm optimizer
    optimizer = funsearch.GAOptimizer(population_size=population_size)

    # Define the fitness function to minimize route distance
    def fitness_function(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm optimizer
    best_route = optimizer.optimize(fitness_function, num_generations)

    return best_route
