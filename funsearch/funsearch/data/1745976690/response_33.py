def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using genetic algorithms."""

    # Define the fitness function
    def fitness(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Create the genetic algorithm optimizer
    optimizer = funsearch.GA(fitness, n_population=100, n_generations=100)

    # Run the optimization
    best_route = optimizer.solve()

    # Return the best route
    return best_route
