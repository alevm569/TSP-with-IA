def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithms."""

    # Initialize population
    population = funsearch.initialize_population(_distances)

    # Run genetic algorithm
    best_route = funsearch.genetic_algorithm(_distances, population)

    return best_route
