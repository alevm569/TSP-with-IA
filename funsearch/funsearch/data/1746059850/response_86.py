def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use genetic algorithm to find the best route
    population_size = 100
    num_generations = 100
    crossover_probability = 0.8
    mutation_probability = 0.2

    # Create a genetic algorithm object
    ga = funsearch.GeneticAlgorithm(
        population_size=population_size,
        num_generations=num_generations,
        crossover_probability=crossover_probability,
        mutation_probability=mutation_probability,
    )

    # Create a fitness function to evaluate the routes
    def fitness(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm to find the best route
    best_route = ga.evolve(fitness, len(_distances))

    # Return the best route
    return best_route
