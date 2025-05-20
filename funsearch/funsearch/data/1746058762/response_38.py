def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use genetic algorithm to find a good solution
    population_size = 100
    num_generations = 100
    tournament_size = 3
    crossover_rate = 0.8
    mutation_rate = 0.2

    # Create the population
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Evaluate the population
    fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Iterate over generations
    for generation in range(num_generations):

        # Tournament selection
        selected_population = funsearch.tournament_selection(population, fitness_values, tournament_size)

        # Crossover
        offspring = funsearch.crossover(selected_population, crossover_rate)

        # Mutation
        mutated_offspring = funsearch.mutation(offspring, mutation_rate)

        # Evaluate the offspring
        offspring_fitness_values = [calculate_route_distance(route, _distances) for route in offspring]

        # Update the population
        population = funsearch.generational_update(population, offspring, fitness_values, offspring_fitness_values)

        # Get the best route
        best_route_index = np.argmin(fitness_values)
        best_route = population[best_route_index]

        # Print the best route for each generation
        print(f"Generation {generation}: {calculate_route_distance(best_route, _distances)}")

    # Return the best route
    return best_route
