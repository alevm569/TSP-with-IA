def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v1` using genetic algorithms.

    Uses a genetic algorithm to search for the best route.
    """

    # Define the chromosome (route) representation.
    chromosome_length = len(_distances)
    tournament_size = 3
    population_size = 50
    mutation_rate = 0.1

    # Initialize the population.
    population = [np.random.permutation(chromosome_length) for _ in range(population_size)]

    # Evaluate the fitness of each route.
    fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Run the genetic algorithm.
    for generation in range(100):
        # Selection.
        parents = funsearch.tournament_selection(population, fitness_values, tournament_size)

        # Crossover.
        offspring = funsearch.crossover(parents, chromosome_length)

        # Mutation.
        funsearch.mutate(offspring, chromosome_length, mutation_rate)

        # Evaluation.
        offspring_fitness = [calculate_route_distance(route, _distances) for route in offspring]

        # Selection of the best routes.
        population = funsearch.genetic_selection(population, fitness_values, offspring, offspring_fitness, population_size)

        # Update the fitness values.
        fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Return the best route found.
    best_route_index = np.argmin(fitness_values)
    return population[best_route_index]
