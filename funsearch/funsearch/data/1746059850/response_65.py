def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Use the nearest neighbor heuristic to generate an initial route.
    start_city = 0
    current_city = start_city
    route = [current_city]

    while len(route) < len(_distances):
        nearest_city = -1
        min_distance = float('inf')

        for i in range(len(_distances)):
            if i not in route:
                distance = _distances[current_city][i]
                if distance < min_distance:
                    nearest_city = i
                    min_distance = distance

        route.append(nearest_city)
        current_city = nearest_city

    # Close the route by adding the starting city.
    route.append(start_city)

    return tuple(route)
