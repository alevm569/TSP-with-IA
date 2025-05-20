def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic or combine multiple heuristics to improve performance.
    # Consider using a hybrid approach that combines different strategies.

    # Example heuristic: Use a combination of nearest neighbor and cheapest insertion
    num_cities = len(_distances)
    route = list(range(num_cities))
    np.random.shuffle(route)

    # Use nearest neighbor to fill in the route
    unvisited = set(route)
    current_city = route[0]
    while len(unvisited) > 0:
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)
        current_city = nearest_city

    # Use cheapest insertion to refine the route
    for i in range(num_cities):
        for j in range(i + 1, num_cities):
            if _distances[route[i]][route[j]] < _distances[route[i]][route[(j + 1) % num_cities]]:
                route[i], route[j] = route[j], route[i]

    # Return the best route as a tuple of city indices.
    return tuple(route)
