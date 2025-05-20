def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Use a hybrid approach that combines nearest neighbor and 2-opt heuristics
    # Initialize the route using nearest neighbor
    route = nearest_neighbor(_distances)

    # Apply 2-opt optimization to improve the route
    route = two_opt(route, _distances)

    return route

# Implement the nearest neighbor and 2-opt heuristics here
