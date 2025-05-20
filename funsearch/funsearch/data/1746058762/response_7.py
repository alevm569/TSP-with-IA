def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize population
    population = [random_route(_distances) for _ in range(100)]

    # Iterate until convergence
    while True:
        # Evaluate population
        fitness_values = [calculate_route_distance(route, _distances) for route in population]

        # Select best routes
        best_routes = np.argsort(fitness_values)[:3]

        # Create new population by combining and mutating best routes
        new_population = []
        for i in best_routes:
            for j in best_routes:
                if i != j:
                    new_route = combine_routes(population[i], population[j], _distances)
                    new_population.append(mutate_route(new_route, _distances))

        # Replace population with new population
        population = new_population

        # Check for convergence
        if fitness_values[best_routes[0]] < 1e-6:
            break

    # Return best route
    return population[best_routes[0]]
