def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Apply a two-opt heuristic to improve the initial solution.
    best_route = find_best_route_v1(_distances)
    improved_route = two_opt(best_route, _distances)

    # If the two-opt heuristic improves the solution, return the improved route.
    if calculate_route_distance(improved_route, _distances) < calculate_route_distance(best_route, _distances):
        return improved_route

    # Otherwise, return the original best route.
    return best_route


def two_opt(route: tuple[int, ...], distances: np.ndarray) -> tuple[int, ...]:
    """
    Two-opt heuristic for improving a TSP route.

    Parameters:
    route (tuple[int, ...]): The current route.
    distances (np.ndarray): A square matrix of distances between cities.

    Returns:
    tuple[int, ...]: The improved route.
    """

    # Iterate over all pairs of cities in the route.
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            # Create a new route by reversing the segment between cities i and j.
            new_route = route[:i] + route[i:j+1][::-1] + route[j+1:]

            # Calculate the distance of the new route.
            new_distance = calculate_route_distance(new_route, distances)

            # If the new route is shorter, return it.
            if new_distance < calculate_route_distance(route, distances):
                return new_route

    # No improvement was found, return the original route.
    return route
