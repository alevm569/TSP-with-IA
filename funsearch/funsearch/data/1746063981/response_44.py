def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Initialize variables
    num_cities = len(_distances)
    best_route = None
    best_distance = float('inf')

    # Use a genetic algorithm
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.2

    # Create a population of routes
    population = [generate_random_route(num_cities) for _ in range(population_size)]

    # Evolve the population
    for generation in range(num_generations):
        # Evaluate the fitness of each route
        fitness_values = [calculate_route_distance(route, _distances) for route in population]

        # Select parents for crossover
        parents = tournament_selection(population, fitness_values)

        # Create a new population of offspring
        offspring = []
        for i in range(population_size):
            parent1, parent2 = parents[i]
            if np.random.rand() < crossover_rate:
                offspring.append(crossover(parent1, parent2))
            else:
                offspring.append(mutation(parent1))

        # Replace the population with the new offspring
        population = offspring

        # Find the best route in the current population
        best_route_in_population = population[np.argmin(fitness_values)]
        best_distance_in_population = fitness_values[np.argmin(fitness_values)]

        # Update the best route if necessary
        if best_distance_in_population < best_distance:
            best_distance = best_distance_in_population
            best_route = best_route_in_population

    # Return the best route found
    return best_route
