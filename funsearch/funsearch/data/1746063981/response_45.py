def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Use a genetic algorithm to explore different route permutations.
    population_size = 50
    num_generations = 100

    # Define the fitness function to minimize route distance.
    def fitness_function(route):
        return calculate_route_distance(route, _distances)

    # Create the genetic algorithm object.
    ga = funsearch.GeneticAlgorithm(population_size, num_generations, fitness_function)

    # Run the genetic algorithm to find the best route.
    best_route = ga.run()

    return best_route
