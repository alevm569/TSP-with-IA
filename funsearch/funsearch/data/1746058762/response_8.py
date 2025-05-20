def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a genetic algorithm to generate candidate routes.
    population_size = 100
    generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create a population of random routes.
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Evaluate the fitness of each route in the population.
    fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Iterate over generations.
    for generation in range(generations):
        # Select parents based on their fitness.
        parents = np.random.choice(population_size, size=population_size, replace=True, p=fitness_values / np.sum(fitness_values))

        # Create children by crossing over parents.
        children = []
        for i in range(population_size):
            parent1 = parents[i]
            parent2 = parents[(i + 1) % population_size]
            crossover_point = np.random.randint(1, len(_distances) - 1)
            child = np.concatenate((parent1[:crossover_point], parent2[crossover_point:]))

        # Mutate children by swapping two cities.
        for child in children:
            mutation_point1 = np.random.randint(0, len(_distances))
            mutation_point2 = np.random.randint(0, len(_distances))
            child[mutation_point1], child[mutation_point2] = child[mutation_point2], child[mutation_point1]

        # Evaluate the fitness of each child.
        child_fitness_values = [calculate_route_distance(route, _distances) for route in children]

        # Replace the worst routes in the population with the new children.
        worst_indices = np.argsort(fitness_values)[:population_size - len(children)]
        population[worst_indices] = children

        # Update the fitness values.
        fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Return the route with the lowest fitness value.
    best_route_index = np.argmin(fitness_values)
    return population[best_route_index]
