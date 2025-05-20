def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithm."""

    # Create a population of candidate routes using genetic algorithm
    population = funsearch.population(
        len(_distances),
        funsearch.random_route,
        funsearch.crossover,
        funsearch.mutation,
        funsearch.fitness_function(_distances),
        population_size=100,
        generations=100,
        verbose=True
    )

    # Find the best route from the population
    best_route = population.best_candidate

    return best_route
