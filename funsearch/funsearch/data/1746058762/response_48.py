def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using metaheuristics."""

    # Initialize a metaheuristic optimizer
    optimizer = funsearch.MetaheuristicOptimizer()

    # Define the fitness function for the TSP problem
    def fitness_function(route: tuple[int, ...]) -> float:
        return calculate_route_distance(route, _distances)

    # Run the metaheuristic optimization
    best_route = optimizer.optimize(fitness_function, len(_distances))

    return best_route
