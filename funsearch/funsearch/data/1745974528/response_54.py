def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Use a genetic algorithm to find the best route
    population_size = 100
    num_generations = 100
    mutation_rate = 0.1
    crossover_rate = 0.8

    # Create a population of routes
    population = funsearch.generate_population(len(_distances), population_size)

    # Evaluate the fitness of each route
    fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Evolve the population for num_generations
    for generation in range(num_generations):
        # Selection
        parents = funsearch.selection(population, fitness_values)

        # Crossover
        offspring = funsearch.crossover(parents, crossover_rate)

        # Mutation
        funsearch.mutation(offspring, mutation_rate)

        # Evaluate the fitness of the offspring
        offspring_fitness_values = [calculate_route_distance(route, _distances) for route in offspring]

        # Replace the worst routes in the population with the offspring
        population = funsearch.replace_worst(population, fitness_values, offspring, offspring_fitness_values)

        # Update the fitness values
        fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Return the best route
    best_route_index = np.argmin(fitness_values)
    return population[best_route_index]


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
