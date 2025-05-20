def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route.
    # Define the fitness function to minimize the total route distance.
    def fitness_function(route):
        return calculate_route_distance(route, _distances)

    # Initialize the population of routes.
    population = funsearch.population(len(_distances), fitness_function)

    # Run the genetic algorithm for a specified number of generations.
    generations = 100
    for generation in range(generations):
        population = funsearch.evolve(population, _distances)

    # Return the best route found by the genetic algorithm.
    return population.best_individual.route
