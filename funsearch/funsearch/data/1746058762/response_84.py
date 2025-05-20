def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use the nearest neighbor heuristic to generate an initial route
    current_city = 0
    route = [current_city]

    while len(route) < len(_distances):
        # Find the closest unvisited city
        distances = _distances[current_city][~np.isin(np.arange(len(_distances)), route)]
        next_city = np.argmin(distances)

        # Add the next city to the route
        route.append(next_city)
        current_city = next_city

    # Close the route by returning to the starting city
    route.append(route[0])

    return tuple(route)
