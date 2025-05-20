def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2`.

    This function implements a hybrid heuristic that combines the nearest neighbor and 2-opt heuristics.
    """

    # Run nearest neighbor to find an initial tour
    tour = nearest_neighbor(_distances)

    # Apply 2-opt local search to improve the tour
    tour = two_opt(tour, _distances)

    return tour
