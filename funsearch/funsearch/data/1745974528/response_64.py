def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a metaheuristic algorithm, such as simulated annealing or genetic algorithm

    # Initialize a population of candidate routes
    population = initialize_population()

    # Iterate until convergence or maximum number of iterations reached
    while True:
        # Evaluate the fitness of each route in the population
        fitness_values = evaluate_population(population, _distances)

        # Select the best route based on fitness
        best_route = population[np.argmin(fitness_values)]

        # Check for convergence or maximum iterations reached
        if is_converged(fitness_values) or max_iterations_reached():
            break

        # Create a new population of candidate routes by applying mutation and crossover operations
        population = create_new_population(population)

    # Return the best route found
    return best_route
