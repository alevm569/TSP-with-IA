def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route.
    population_size = 100
    generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create a population of routes.
    population = [random_route(_distances) for _ in range(population_size)]

    # Evaluate the fitness of each route in the population.
    fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Iterate over the generations.
    for generation in range(generations):
        # Select the best routes in the population.
        best_routes = sorted(population, key=calculate_route_distance, reverse=False)[:population_size // 2]

        # Create a new population of routes.
        new_population = []

        # Perform crossover and mutation.
        for _ in range(population_size):
            # Select two parents from the best routes.
            parent1, parent2 = random.sample(best_routes, 2)

            # Perform crossover.
            crossover_point = random.randint(1, len(_distances) - 2)
            child = parent1[:crossover_point] + parent2[crossover_point:]

            # Perform mutation.
            if random.random() < mutation_rate:
                mutation_point = random.randint(0, len(_distances) - 1)
                child[mutation_point] = random.randint(0, len(_distances) - 1)

            new_population.append(child)

        population = new_population

    # Return the best route found.
    best_route = sorted(population, key=calculate_route_distance, reverse=False)[0]
    return best_route
