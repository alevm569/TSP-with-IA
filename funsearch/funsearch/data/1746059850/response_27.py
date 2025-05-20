def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Use the nearest neighbor heuristic to generate an initial solution.
    start_city = 0
    current_city = start_city
    route = [current_city]

    while len(route) < len(_distances):
        nearest_city = np.argmin(_distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Add the start city to the end of the route.
    route.append(start_city)

    return tuple(route)
