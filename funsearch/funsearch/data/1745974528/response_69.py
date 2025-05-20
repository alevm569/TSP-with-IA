def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Perform genetic algorithm to find the best route
    population_size = 100
    num_generations = 100
    tournament_size = 5

    # Create a genetic algorithm object
    ga = funsearch.GeneticAlgorithm(
        population_size=population_size,
        num_generations=num_generations,
        tournament_size=tournament_size,
        crossover_rate=0.8,
        mutation_rate=0.2,
    )

    # Create a fitness function to evaluate the quality of each route
    def fitness_function(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm to find the best route
    best_route = ga.solve(fitness_function)

    # Return the best route
    return best_route
