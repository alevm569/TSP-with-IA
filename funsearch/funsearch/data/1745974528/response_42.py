def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    Hybrid heuristic combines two approaches:
        - Genetic Algorithm (GA): For global search and population diversity.
        - Tabu Search (TS): For local search and avoiding stuck solutions.

    The GA's population size and TS's neighborhood size can be tuned based on the problem size and complexity.
    """

    # Genetic Algorithm parameters
    population_size = 100
    generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.2

    # Tabu Search parameters
    tabu_tenure = 10
    neighborhood_size = 5

    # Initialize GA population
    population = funsearch.generate_random_population(population_size, len(_distances))

    # Run GA for multiple generations
    for generation in range(generations):
        # Evaluate fitness of each route in the population
        fitnesses = [calculate_route_distance(route, _distances) for route in population]

        # Select parent routes for crossover
        parents = funsearch.tournament_selection(population, fitnesses)

        # Perform crossover and mutation
        offspring = funsearch.crossover(parents, crossover_rate)
        offspring = funsearch.mutate(offspring, mutation_rate)

        # Add new routes to the population
        population.extend(offspring)

        # Run Tabu Search on the best routes in the population
        best_routes = funsearch.tabu_search(population, fitnesses, neighborhood_size, tabu_tenure)

        # Update the population with the best routes found by Tabu Search
        population = best_routes

    # Return the best route found by the hybrid heuristic
    best_route_index = np.argmin([calculate_route_distance(route, _distances) for route in population])
    return population[best_route_index]
