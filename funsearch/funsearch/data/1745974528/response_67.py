def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithm."""

    # Create a genetic algorithm object
    ga = funsearch.GeneticAlgorithm(
        population_size=100,
        mutation_rate=0.01,
        crossover_rate=0.8,
        tournament_size=3,
        elitism=True,
    )

    # Define the fitness function
    def fitness_function(route: np.ndarray) -> float:
        return -calculate_route_distance(route, _distances)

    # Run the genetic algorithm
    best_route = ga.run(fitness_function, _distances)

    # Return the best route as a tuple of integers
    return tuple(best_route)
