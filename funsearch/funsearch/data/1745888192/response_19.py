def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Apply a hybrid heuristic that combines different approaches, such as:
    # - Nearest neighbor to generate an initial route
    # - Cheapest insertion to refine the route
    # - Local search to optimize the route

    # Example using nearest neighbor:
    current_city = 0
    route = [current_city]
    while len(route) < len(_distances):
        nearest_city = np.argmin(_distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Add the return to the starting city
    route.append(route[0])

    return tuple(route)
