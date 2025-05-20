def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement an improved heuristic or combine multiple heuristics to improve performance.
    # Consider using a hybrid approach that combines different strategies.

    # Use the 2-opt heuristic to improve the route.
    best_route = find_best_route_v2(_distances)
    best_distance = calculate_route_distance(best_route, _distances)

    for i in range(len(best_route)):
        for j in range(i + 2, len(best_route)):
            new_route = swap_cities(best_route, i, j)
            new_distance = calculate_route_distance(new_route, _distances)
            if new_distance < best_distance:
                best_route = new_route
                best_distance = new_distance

    # Return the best route as a tuple of city indices.
    return best_route


def swap_cities(route: tuple[int, ...], i: int, j: int) -> tuple[int, ...]:
    """Swaps two cities in a route."""
    return route[:i] + route[j:i:-1] + route[j + 1:]
