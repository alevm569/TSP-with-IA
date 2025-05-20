def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Apply a two-opt heuristic to improve route quality.
    route = funsearch.two_opt(_distances)

    # Ensure the route includes all cities exactly once and returns to the starting point.
    assert len(set(route)) == len(_distances)
    assert route[0] == route[-1]

    return route
