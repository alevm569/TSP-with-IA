def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a combination of nearest neighbor and 2-opt heuristics
    route = funsearch.nearest_neighbor(_distances)
    funsearch.two_opt(route, _distances)

    return route
