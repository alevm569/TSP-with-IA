def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route
    population_size = 100
    mutation_rate = 0.1
    tournament_size = 5

    # Create a genetic algorithm object
    ga = funsearch.GA(population_size, mutation_rate, tournament_size)

    # Set the fitness function
    ga.fitness_function = calculate_route_distance

    # Run the genetic algorithm
    best_route = ga.run(population=_distances)

    # Return the best route
    return best_route
