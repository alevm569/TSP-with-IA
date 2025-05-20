def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use a hybrid approach that combines multiple heuristics.
    # First, use the nearest neighbor heuristic to find an initial route.
    current_city = 0
    route = [current_city]
    unvisited_cities = set(range(len(_distances)))
    unvisited_cities.remove(current_city)

    while unvisited_cities:
        nearest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)
        current_city = nearest_city

    # Next, use the 2-opt heuristic to improve the route.
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:]
            new_route[i:j+1] = new_route[j:i:-1]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    # Return the best route as a tuple of city indices.
    return tuple(route)
