def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Use a metaheuristic algorithm, such as simulated annealing or genetic algorithms.
    # Initialize a population of random routes.
    population = [np.random.permutation(len(_distances)) for _ in range(100)]

    # Define a fitness function to evaluate the total distance of a route.
    def fitness(route):
        return calculate_route_distance(route, _distances)

    # Run the metaheuristic algorithm for 1000 iterations.
    for i in range(1000):
        # Select two routes from the population.
        route1, route2 = np.random.choice(population, size=2, replace=False)

        # Perform a crossover operation to create a new route.
        crossover_point = np.random.randint(1, len(route1))
        new_route = np.concatenate((route1[:crossover_point], route2[crossover_point:]))

        # Evaluate the new route.
        new_fitness = fitness(new_route)

        # If the new route is better than one of the parents, replace it.
        if new_fitness < fitness(route1) or new_fitness < fitness(route2):
            population.append(new_route)

    # Return the route with the lowest fitness.
    return min(population, key=fitness)
