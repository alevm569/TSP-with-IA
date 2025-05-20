def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithms."""

    # Define the fitness function
    def fitness(route: list[int]) -> float:
        total_distance = calculate_route_distance(route, _distances)
        return 1 / total_distance

    # Create the genetic algorithm
    population_size = 100
    num_generations = 100
    crossover_rate = 0.8
    mutation_rate = 0.1

    # Create the initial population
    population = funsearch.random_population(population_size, len(_distances))

    # Run the genetic algorithm
    best_route = funsearch.genetic_algorithm(
        population,
        fitness,
        crossover_rate,
        mutation_rate,
        num_generations,
    )

    return best_route
