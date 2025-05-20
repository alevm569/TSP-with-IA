def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v1`.

    Uses a hybrid approach combining nearest neighbor and 2-opt heuristics.
    """

    # Perform nearest neighbor to get an initial route
    initial_route = nearest_neighbor(_distances)

    # Apply 2-opt optimization to refine the route
    best_route = two_opt(initial_route, _distances)

    return best_route
