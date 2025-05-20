def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Genetic Algorithm (GA) with elitism
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.2

    # Create a GA object
    ga = funsearch.GA(population_size, num_generations, crossover_rate, mutation_rate)

    # Define the fitness function
    def fitness(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Run the GA
    best_route = ga.run(fitness, len(_distances))

    # Return the best route
    return best_route
