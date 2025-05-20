def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using metaheuristics.

    Use metaheuristics such as simulated annealing, ant colony optimization,
    genetic algorithms, or other novel metaheuristic approaches.

    Routes must include all cities exactly once and return to the starting point.
    """

    # Initialize metaheuristic algorithm
    sa = funsearch.SimulatedAnnealing(
        funsearch.RouteOptimization(_distances),
        funsearch.RandomMove(),
        funsearch.CoolSchedule(1000, 0.9),
    )

    # Run metaheuristic algorithm
    best_route = sa.run()

    # Return the best route found
    return best_route.route
