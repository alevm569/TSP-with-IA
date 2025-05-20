def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use the nearest neighbor heuristic to generate an initial route.
    current_city = 0
    route = [current_city]

    while len(route) < len(_distances):
        # Find the city that is closest to the current city but not already in the route.
        best_city = None
        best_distance = float('inf')

        for i in range(len(_distances)):
            if i not in route:
                distance = _distances[current_city][i]
                if distance < best_distance:
                    best_city = i
                    best_distance = distance

        # Add the best city to the route.
        route.append(best_city)

        # Update the current city.
        current_city = best_city

    # Return to the starting city.
    route.append(route[0])

    return tuple(route)
