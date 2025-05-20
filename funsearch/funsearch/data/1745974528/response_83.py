def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Implement your new heuristic here.
    # Consider using a combination of existing heuristics or developing your own.

    # Example heuristic:
    # - Start from an initial city.
    # - Find the city with the minimum distance to the current city.
    # - Add the next city to the route.
    # - Repeat until all cities have been visited.
    # - Return to the starting city.

    # Example implementation:
    route = [0]
    visited = {0}
    while len(visited) < len(_distances):
        current_city = route[-1]
        next_city = np.argmin(_distances[current_city][[i for i in range(len(_distances)) if i not in visited]])
        route.append(next_city)
        visited.add(next_city)

    return tuple(route)
