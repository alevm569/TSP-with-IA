def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Perform a nearest neighbor search to find an initial route.
    start_city = 0
    current_city = start_city
    route = [current_city]

    while len(route) < len(_distances):
        # Find the closest unvisited city to the current city.
        closest_city = np.argmin(_distances[current_city][~np.isin(np.arange(len(_distances)), route)])
        route.append(closest_city)
        current_city = closest_city

    # Add the starting city back to the route.
    route.append(start_city)

    return tuple(route)
