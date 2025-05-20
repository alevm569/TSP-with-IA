def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithm."""

    # Define the fitness function
    def fitness(route):
        total_distance = 0
        for i in range(len(route)):
            total_distance += _distances[route[i]][route[(i + 1) % len(route)]]
        return total_distance

    # Define the genetic algorithm parameters
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Initialize the population
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Run the genetic algorithm
    for generation in range(num_generations):
        # Selection
        parents = funsearch.tournament_selection(population, fitness)

        # Crossover
        offspring = funsearch.crossover(parents, crossover_rate)

        # Mutation
        funsearch.mutate(offspring, mutation_rate)

        # Evaluation
        fitness_values = [fitness(route) for route in offspring]

        # Selection of the best route
        best_route = offspring[np.argmin(fitness_values)]

        # Update the population
        population = funsearch.elitism(population, offspring, fitness_values)

    # Return the best route
    return best_route
