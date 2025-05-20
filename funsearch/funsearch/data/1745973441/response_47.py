def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Use a genetic algorithm to search for the best route
    population_size = 100
    num_generations = 100
    tournament_size = 5
    crossover_rate = 0.8
    mutation_rate = 0.2

    # Create a population of random routes
    population = np.random.permutation(len(_distances)).reshape(-1, len(_distances))

    # Evaluate the fitness of each route
    fitness = np.apply_along_axis(calculate_route_distance, 1, population, _distances)

    # Iterate until the best route is found
    for generation in range(num_generations):
        # Select parents using tournament selection
        parents = np.random.choice(population.shape[0], population_size, replace=True)
        parents = population[np.random.randint(tournament_size, size=population_size)]

        # Create offspring by crossover and mutation
        offspring = np.empty_like(population)
        for i in range(population_size):
            # Crossover
            if np.random.rand() < crossover_rate:
                parent1 = parents[np.random.randint(tournament_size)]
                parent2 = parents[np.random.randint(tournament_size)]
                crossover_point = np.random.randint(1, len(_distances))
                offspring[i, :crossover_point] = parent1[:crossover_point]
                offspring[i, crossover_point:] = parent2[crossover_point:]

            # Mutation
            if np.random.rand() < mutation_rate:
                mutation_point1 = np.random.randint(len(_distances))
                mutation_point2 = np.random.randint(len(_distances))
                offspring[i, mutation_point1], offspring[i, mutation_point2] = offspring[i, mutation_point2], offspring[i, mutation_point1]

        # Evaluate the fitness of the offspring
        fitness_offspring = np.apply_along_axis(calculate_route_distance, 1, offspring, _distances)

        # Replace the worst routes in the population with the offspring
        worst_indices = np.argsort(fitness)[:population_size - offspring.shape[0]]
        population[worst_indices] = offspring

        # Update the best route
        best_index = np.argmin(fitness)
        best_route = population[best_index]

    return best_route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculate the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
