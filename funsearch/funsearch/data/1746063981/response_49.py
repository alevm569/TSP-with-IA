def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Use a genetic algorithm to find the best route
    # The fitness function should calculate the total distance of the route
    # The genetic algorithm should use crossover, mutation, and elitism to improve the route
    # Genetic Algorithm parameters
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.2
    elite_percentage = 0.2

    # Create a population of routes
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Iterate over generations
    for generation in range(num_generations):
        # Evaluate the fitness of each route
        fitness = [calculate_route_distance(route, _distances) for route in population]

        # Select the fittest routes
        elite_size = int(population_size * elite_percentage)
        elite_routes = population[:elite_size]

        # Create new routes through crossover and mutation
        new_population = []
        for _ in range(population_size - elite_size):
            parent1 = np.random.choice(population)
            parent2 = np.random.choice(population)

            # Crossover
            if np.random.rand() < crossover_rate:
                crossover_point = np.random.randint(1, len(_distances) - 1)
                child = np.concatenate((parent1[:crossover_point], parent2[crossover_point:]))

            # Mutation
            if np.random.rand() < mutation_rate:
                mutation_point1, mutation_point2 = np.random.randint(0, len(_distances), 2)
                child[mutation_point1], child[mutation_point2] = child[mutation_point2], child[mutation_point1]

            new_population.append(child)

        # Replace the worst routes with the new routes
        population = elite_routes + new_population

    # Return the best route
    best_route = population[np.argmin([calculate_route_distance(route, _distances) for route in population])]
    return best_route
