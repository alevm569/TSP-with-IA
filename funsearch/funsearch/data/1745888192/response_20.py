def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route
    population_size = 100
    num_generations = 100
    tournament_size = 5
    crossover_probability = 0.8
    mutation_probability = 0.2

    # Create a fitness function to evaluate the total distance of a route
    def fitness_function(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Create a genetic algorithm object
    ga = funsearch.GA(population_size, fitness_function)

    # Run the genetic algorithm
    best_route = ga.run(num_generations, tournament_size, crossover_probability, mutation_probability)

    # Return the best route found
    return best_route
