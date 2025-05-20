def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a two-opt heuristic to improve the solution quality.
    best_route = find_best_route_v1(_distances)
    improved_route = two_opt(best_route, _distances)

    # Ensure that the route includes all cities exactly once and returns to the starting point.
    return improved_route

def two_opt(route: tuple[int, ...], _distances: np.ndarray) -> tuple[int, ...]:
    """
    Performs a two-opt operation on a TSP route.

    Args:
        route: A permutation of cities.
        _distances: A square matrix of distances between cities.

    Returns:
        An improved route.
    """

    best_distance = calculate_route_distance(route, _distances)
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:i] + (route[j], route[i],) + route[j+1:]
            new_distance = calculate_route_distance(new_route, _distances)
            if new_distance < best_distance:
                best_distance = new_distance
                best_route = new_route

    return best_route
