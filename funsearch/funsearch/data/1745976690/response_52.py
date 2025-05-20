def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize the route with the nearest neighbor strategy
    start_city = 0
    route = [start_city]
    unvisited = set(range(1, len(_distances)))

    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Perform local search to refine the route
    for _ in range(10):
        for i in range(len(route)):
            for j in range(i + 1, len(route)):
                new_route = route[:i] + route[j:i:-1] + route[j + 1:]
                if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                    route = new_route

    return tuple(route)
