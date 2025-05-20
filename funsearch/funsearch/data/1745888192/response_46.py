def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Initialize the route with the first city.
    route = [0]

    # Iterate until all cities are visited.
    while len(route) < len(_distances):
        # Find the city that is closest to the last city in the route.
        min_distance = float('inf')
        for i in range(len(_distances)):
            if i not in route:
                distance = _distances[route[-1]][i]
                if distance < min_distance:
                    min_distance = distance
                    next_city = i

        # Add the next city to the route.
        route.append(next_city)

    # Return the route.
    return tuple(route)
