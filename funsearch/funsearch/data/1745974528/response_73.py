def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Apply the simulated annealing algorithm
    initial_route = np.random.permutation(len(_distances))
    optimal_route, optimal_distance = funsearch.simulated_annealing(initial_route, _distances)

    return optimal_route
