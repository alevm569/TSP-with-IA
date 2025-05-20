def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    This version combines the nearest neighbor heuristic for initialization and a 2-opt local search for optimization.
    """

    # Initialize route using nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(len(_distances))) - {start_city}

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # Perform 2-opt local search to optimize route
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            new_route = route[:i] + route[i+j:j:-1] + route[j+1:]
            if calculate_route_distance(new_route, _distances) < calculate_route_distance(route, _distances):
                route = new_route

    return tuple(route)
