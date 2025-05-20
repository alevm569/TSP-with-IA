def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using local search."""

    # Generate an initial route using the nearest neighbor heuristic
    start_city = 0
    current_city = start_city
    route = [current_city]

    while len(route) < len(_distances):
        nearest_city = np.argmin(_distances[current_city])
        if nearest_city not in route:
            route.append(nearest_city)
            current_city = nearest_city

    # Perform local search to improve the route
    for i in range(100):
        best_distance = calculate_route_distance(route, _distances)

        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                new_route = route[:]
                new_route[i], new_route[j] = new_route[j], new_route[i]
                new_distance = calculate_route_distance(new_route, _distances)

                if new_distance < best_distance:
                    route = new_route
                    best_distance = new_distance

    return route
