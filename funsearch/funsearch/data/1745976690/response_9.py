def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with genetic algorithm."""

    # Define the fitness function
    def fitness(route):
        return -calculate_route_distance(route, _distances)

    # Create the genetic algorithm parameters
    population_size = 100
    generations = 100
    mutation_rate = 0.01

    # Create the genetic algorithm object
    ga = funsearch.GeneticAlgorithm(fitness, population_size, generations, mutation_rate)

    # Run the genetic algorithm
    best_route = ga.run()

    # Return the best route
    return best_route
