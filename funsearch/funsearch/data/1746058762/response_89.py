def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Implement a new heuristic or strategy here.
    # For example, you could use a hybrid of the nearest neighbor and cheapest insertion heuristics.

    # Perform local search on the initial solution to find a better route.
    # You can use the 2-opt or a more sophisticated neighborhood operator.

    # Return the best route found.
    return tuple(best_route)
