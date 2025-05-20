def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic here, for example, using a metaheuristic algorithm like simulated annealing or genetic algorithm.

    # Return the best route found by the heuristic.
    return tuple(np.random.permutation(np.arange(len(_distances))))
