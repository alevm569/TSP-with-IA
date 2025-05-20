def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a genetic algorithm to search for the best route.
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.2

    # Create a population of routes.
    population = [
        funsearch.RandomPermutation(len(_distances))
        for _ in range(population_size)
    ]

    # Run the genetic algorithm.
    for generation in range(num_generations):
        # Evaluate the fitness of each route.
        fitness_values = [
            calculate_route_distance(route, _distances)
            for route in population
        ]

        # Select the best routes.
        best_routes = funsearch.TournamentSelection(population, fitness_values)

        # Create new routes through crossover and mutation.
        offspring = funsearch.CrossoverAndMutation(population, crossover_rate, mutation_rate)

        # Add the new routes to the population.
        population.extend(offspring)

    # Return the best route.
    best_route = population[np.argmin(fitness_values)]
    return best_route
