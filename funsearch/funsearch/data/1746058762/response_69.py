def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 using a hybrid heuristic.

    This function combines the following heuristics:
        - Nearest neighbor
        - Cheapest insertion
        - 2-opt local search

    Args:
        _distances (np.ndarray): A square matrix of distances between cities.

    Returns:
        A tuple representing the best route.
    """

    # Initialize the route using the nearest neighbor heuristic
    route = nearest_neighbor(_distances)

    # Use the cheapest insertion heuristic to refine the route
    route = cheapest_insertion(_distances, route)

    # Apply 2-opt local search to optimize the route
    route = two_opt(_distances, route)

    return route
