def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Perform nearest neighbor search to find an initial route
    start_city = 0
    route = [start_city]
    unvisited_cities = set(range(len(_distances)))
    unvisited_cities.remove(start_city)

    while unvisited_cities:
        current_city = route[-1]
        nearest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # Perform 2-opt optimization to improve the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:i] + route[j:i:-1] + route[j + 1:]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return tuple(route)
