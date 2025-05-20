def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid approach."""

    # Perform nearest neighbor search to generate an initial route.
    start_city = 0
    current_city = start_city
    route = [current_city]
    unvisited_cities = set(range(len(_distances))) - {start_city}

    while unvisited_cities:
        nearest_city = min(unvisited_cities, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)
        current_city = nearest_city

    # Perform local search to improve the route.
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            new_route = route[:]
            new_route[i], new_route[j] = new_route[j], new_route[i]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return route
