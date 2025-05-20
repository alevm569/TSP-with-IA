def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithm."""

    # Define the fitness function
    def fitness(route: tuple[int, ...]) -> float:
        return -calculate_route_distance(route, _distances)

    # Create a genetic algorithm object
    ga = funsearch.GeneticAlgorithm(
        population_size=100,
        num_generations=50,
        fitness_function=fitness,
        crossover_probability=0.8,
        mutation_probability=0.2,
    )

    # Run the genetic algorithm
    ga.run()

    # Return the best route found
    return ga.best_solution
