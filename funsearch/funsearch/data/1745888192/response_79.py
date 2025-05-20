def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a hybrid heuristic that combines the nearest neighbor and cheapest insertion heuristics.
    best_route = funsearch.greedy.hybrid_heuristic(
        _distances,
        funsearch.greedy.nearest_neighbor,
        funsearch.greedy.cheapest_insertion,
    )

    return best_route
