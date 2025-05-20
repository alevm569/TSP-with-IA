def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create a population of candidate routes
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Evaluate the fitness of each route
    fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Iterate over generations
    for generation in range(num_generations):
        # Select parents based on their fitness
        parents = np.random.choice(population_size, size=population_size, p=fitness_values / np.sum(fitness_values))

        # Create offspring through crossover
        offspring = []
        for i in range(population_size):
            parent1 = parents[np.random.randint(population_size)]
            parent2 = parents[np.random.randint(population_size)]
            crossover_point = np.random.randint(len(_distances))
            offspring.append(np.concatenate((parent1[:crossover_point], parent2[crossover_point:])))

        # Mutate offspring
        for route in offspring:
            mutation_point1 = np.random.randint(len(_distances))
            mutation_point2 = np.random.randint(len(_distances))
            route[mutation_point1], route[mutation_point2] = route[mutation_point2], route[mutation_point1]

        # Evaluate the fitness of offspring
        fitness_values = [calculate_route_distance(route, _distances) for route in offspring]

        # Update population
        population = np.concatenate((population, offspring))
        fitness_values = np.concatenate((fitness_values, fitness_values))

        # Select the best routes
        best_routes = population[np.argsort(fitness_values)[:population_size]]

    # Return the best route
    return best_routes[0]
