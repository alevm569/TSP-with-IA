def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid approach."""

    # Initial solution using nearest neighbor
    start_city = 0
    current_city = start_city
    route = [current_city]

    while len(route) < len(_distances):
        nearest_city = np.argmin(_distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Local search optimization
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            # Swap two cities in the route
            route[i], route[j] = route[j], route[i]

            # Check if the new route is better
            if calculate_route_distance(route, _distances) < calculate_route_distance(route, _distances):
                break

    return tuple(route)
