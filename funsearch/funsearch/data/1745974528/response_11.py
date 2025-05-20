def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using metaheuristics."""

    # Initialize metaheuristic algorithm
    algorithm = funsearch.SimulatedAnnealing(
        initial_solution=funsearch.RandomSolution(),
        fitness_function=calculate_route_distance,
        distance_matrix=_distances,
    )

    # Run metaheuristic algorithm
    best_route = algorithm.run()

    return best_route.solution
