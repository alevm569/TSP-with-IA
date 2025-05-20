def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a metaheuristic algorithm, such as simulated annealing or ant colony optimization.
    # These algorithms can find near-optimal solutions to the TSP problem.

    # Example using simulated annealing:
    from funsearch import SimulatedAnnealing

    def fitness_function(route):
        return calculate_route_distance(route, _distances)

    sa = SimulatedAnnealing(fitness_function, initial_state=list(range(len(_distances))))
    sa.optimize()

    # Return the best route found by the algorithm.
    return tuple(sa.best_state)
