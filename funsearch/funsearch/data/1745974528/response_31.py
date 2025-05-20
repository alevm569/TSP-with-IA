def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a genetic algorithm."""

    # Define a fitness function to minimize the total route distance
    def fitness_function(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Create a population of routes using a genetic algorithm
    population = funsearch.generate_population(
        num_routes=100,
        route_length=len(_distances),
        fitness_function=fitness_function
    )

    # Evolve the population using genetic operators
    population = funsearch.evolve_population(population, _distances)

    # Return the best route found
    return population[0]
