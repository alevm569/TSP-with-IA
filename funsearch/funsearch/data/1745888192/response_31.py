def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find a good solution.
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create a population of random routes.
    population = [np.random.permutation(len(_distances)) for _ in range(population_size)]

    # Evaluate the fitness of each route in the population.
    fitness = [calculate_route_distance(route, _distances) for route in population]

    # Iterate over generations.
    for generation in range(num_generations):
        # Select parents for reproduction.
        parents = funsearch.tournament_selection(population, fitness, 2)

        # Create offspring through crossover.
        offspring = funsearch.crossover(parents, _distances, crossover_rate)

        # Mutate offspring.
        funsearch.mutation(offspring, _distances, mutation_rate)

        # Evaluate the fitness of the offspring.
        fitness = [calculate_route_distance(route, _distances) for route in offspring]

        # Select the best routes to keep.
        population = funsearch.elitism(population, fitness, population_size)

    # Return the route with the best fitness.
    best_route = population[np.argmin(fitness)]
    return best_route


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
