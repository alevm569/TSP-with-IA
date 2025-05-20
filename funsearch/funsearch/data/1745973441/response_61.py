def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`.

    Uses a hybrid heuristic that combines local search with a 2-opt heuristic.
    """

    # Create an initial route using the nearest neighbor heuristic.
    route = nearest_neighbor_heuristic(_distances)

    # Run local search to improve the route.
    route = local_search(route, _distances)

    # Perform 2-opt swaps to further optimize the route.
    route = two_opt(route, _distances)

    return route
