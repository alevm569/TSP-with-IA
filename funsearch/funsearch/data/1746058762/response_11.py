def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use the nearest neighbor heuristic to find an initial route.
    start_city = 0
    current_city = start_city
    route = [current_city]

    while len(route) < len(_distances):
        next_city = np.argmin(_distances[current_city])
        if next_city not in route:
            route.append(next_city)
            current_city = next_city

    # Close the route by adding the starting city again.
    route.append(start_city)

    return tuple(route)
