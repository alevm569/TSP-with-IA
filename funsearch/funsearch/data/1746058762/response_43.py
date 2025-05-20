def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithm."""

    # Define the fitness function
    def fitness_function(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Create the genetic algorithm object
    ga = funsearch.GeneticAlgorithm(fitness_function)

    # Set the population size and number of generations
    ga.population_size = 100
    ga.num_generations = 1000

    # Run the genetic algorithm
    best_route = ga.run()

    # Return the best route found
    return best_route
