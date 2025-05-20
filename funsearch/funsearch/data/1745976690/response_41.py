def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 using a hybrid heuristic.

    This version combines the nearest neighbor heuristic for initialization and the 2-opt local search heuristic for optimization.
    """

    # Initialize the route using the nearest neighbor heuristic
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(len(_distances))) - {current_city}

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Optimize the route using the 2-opt local search heuristic
    for _ in range(100):  # Run for up to 100 iterations
        for i in range(len(route)):
            for j in range(i + 2, len(route)):
                if _distances[route[i]][route[j]] < _distances[route[i]][route[i+1]] + _distances[route[j]][route[j-1]]:
                    route[i+1:j] = route[j:i:-1]

    return tuple(route)
