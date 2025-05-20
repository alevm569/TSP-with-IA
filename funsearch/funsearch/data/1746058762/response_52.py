def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to search for the best route.
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create a population of routes.
    population = [np.random.permutation(np.arange(len(_distances))) for _ in range(population_size)]

    # Evaluate the fitness of each route.
    fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Iterate until the best route is found.
    for generation in range(num_generations):
        # Select parents for reproduction.
        parents = np.random.choice(population_size, size=population_size, p=fitness_values / np.sum(fitness_values))

        # Create new offspring through crossover.
        offspring = []
        for i in range(population_size):
            parent1 = population[parents[i]]
            parent2 = population[parents[(i + 1) % population_size]]
            crossover_point = np.random.randint(1, len(_distances) - 1)
            offspring.append(np.concatenate((parent1[:crossover_point], parent2[crossover_point:])))

        # Mutate the offspring.
        for route in offspring:
            mutation_point1 = np.random.randint(0, len(_distances))
            mutation_point2 = np.random.randint(0, len(_distances))
            route[mutation_point1], route[mutation_point2] = route[mutation_point2], route[mutation_point1]

        # Evaluate the fitness of the offspring.
        fitness_values_offspring = [calculate_route_distance(route, _distances) for route in offspring]

        # Replace the worst routes with the offspring.
        worst_indices = np.argsort(fitness_values)[:population_size - len(offspring)]
        population[worst_indices] = offspring

        # Update the fitness values.
        fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Return the best route.
    best_route_index = np.argmin(fitness_values)
    return population[best_route_index]
