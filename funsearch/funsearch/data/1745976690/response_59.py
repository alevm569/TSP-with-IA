def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route.
    population_size = 100
    num_generations = 50
    tournament_size = 5

    # Create a genetic algorithm object.
    ga = funsearch.GeneticAlgorithm(population_size, num_generations, tournament_size)

    # Create a fitness function.
    def fitness_function(route):
        return calculate_route_distance(route, _distances)

    # Run the genetic algorithm.
    best_route = ga.run(fitness_function)

    # Return the best route.
    return best_route
