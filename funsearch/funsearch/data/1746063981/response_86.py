def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Use nearest neighbor to find an initial route
    current_city = 0
    route = [current_city]

    while len(route) < len(_distances):
        # Find the nearest unvisited city
        nearest_city = np.argmin(_distances[current_city][~np.isin(np.arange(len(_distances)), route)])
        route.append(nearest_city)
        current_city = nearest_city

    # Use 2-opt to improve the route
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            new_route = route[:]
            new_route[i:j] = new_route[j:i:-1]

            # Check if the new route is shorter
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return route
