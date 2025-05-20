def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a hybrid heuristic."""

    # Use nearest neighbor to generate an initial route
    current_city = 0
    route = [current_city]

    while len(route) < len(_distances):
        nearest_city = -1
        min_distance = math.inf

        for i in range(len(_distances)):
            if i not in route:
                distance = _distances[current_city][i]
                if distance < min_distance:
                    nearest_city = i
                    min_distance = distance

        current_city = nearest_city
        route.append(current_city)

    # Perform local search to refine the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:]
            new_route[i], new_route[j] = new_route[j], new_route[i]

            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return route
