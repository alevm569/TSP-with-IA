def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use the nearest neighbor heuristic to generate an initial route
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(len(_distances))) - {current_city}

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Add the return trip to the starting city
    route.append(route[0])

    # Use the 2-opt local search algorithm to improve the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            new_route = route[:i] + route[j:i:-1] + route[j+1:]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return route
