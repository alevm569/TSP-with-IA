def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a metaheuristic algorithm, such as simulated annealing or ant colony optimization.
    # These algorithms can efficiently search the solution space and find better routes.

    # Example using simulated annealing:
    from funsearch.metaheuristics.sa import SimulatedAnnealing

    def route_distance(route):
        return calculate_route_distance(route, _distances)

    sa = SimulatedAnnealing(route_distance)
    best_route = sa.optimize()

    return best_route
