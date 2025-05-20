def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with a hybrid heuristic."""

    # Initialize population with a random set of routes
    population = funsearch.population(size=100)

    # Create a fitness function to evaluate routes based on total distance
    fitness_function = funsearch.fitness_function(
        evaluate_route,
        minimize=True,
    )

    # Run the genetic algorithm for 100 generations
    best_route = funsearch.genetic_algorithm(
        population=population,
        fitness_function=fitness_function,
        generations=100,
        tournament_size=3,
        crossover_rate=0.8,
        mutation_rate=0.1,
    )

    # Return the best route found
    return best_route
