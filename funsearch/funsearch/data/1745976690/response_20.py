def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using genetic algorithm."""

    # Define the population size and number of generations
    population_size = 100
    num_generations = 100

    # Create a genetic algorithm object
    ga = funsearch.GeneticAlgorithm(population_size=population_size, tournament_size=3)

    # Define the fitness function
    def fitness(route: tuple[int, ...]) -> float:
        return -calculate_route_distance(route, _distances)

    # Run the genetic algorithm
    best_route = ga.run(fitness, num_generations, initial_population=None)

    # Return the best route
    return best_route
