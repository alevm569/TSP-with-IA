def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a genetic algorithm to find the best route
    num_cities = len(_distances)
    population_size = 100
    num_generations = 100
    crossover_probability = 0.8
    mutation_probability = 0.2

    # Create a population of routes
    population = [np.random.permutation(num_cities) for _ in range(population_size)]

    # Evaluate the fitness of each route
    fitness_values = [calculate_route_distance(route, _distances) for route in population]

    # Iterate over generations
    for generation in range(num_generations):

        # Select the best routes
        best_routes = population[np.argsort(fitness_values)[:population_size // 2]]

        # Create new routes through crossover and mutation
        new_routes = []
        for i in range(population_size):
            parent1 = np.random.choice(best_routes)
            parent2 = np.random.choice(best_routes)
            crossover_point = np.random.randint(num_cities)
            child = np.concatenate((parent1[:crossover_point], parent2[crossover_point:]))
            mutation_mask = np.random.rand(num_cities) < mutation_probability
            child[mutation_mask] = np.random.permutation(np.delete(np.arange(num_cities), child))
            new_routes.append(child)

        # Evaluate the fitness of new routes
        new_fitness_values = [calculate_route_distance(route, _distances) for route in new_routes]

        # Replace the worst routes with new routes
        population = new_routes
        fitness_values = new_fitness_values

    # Return the best route found
    return population[np.argmin(fitness_values)]


def calculate_route_distance(route: tuple[int, ...], distances: np.ndarray) -> float:
    """Calculates the total distance of a route."""
    total_distance = 0
    for i in range(len(route)):
        total_distance += distances[route[i]][route[(i + 1) % len(route)]]
    return total_distance
