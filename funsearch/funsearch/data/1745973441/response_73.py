def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Perform nearest neighbor search to generate an initial route
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(len(_distances)))
    remaining_cities.remove(current_city)

    while remaining_cities:
        # Find the city with the shortest distance from the current city
        closest_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        route.append(closest_city)
        remaining_cities.remove(closest_city)
        current_city = closest_city

    # Use 2-opt heuristic to improve the route
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            # Swap cities i and j and calculate the distance difference
            distance_difference = _distances[route[i]][route[j]] - _distances[route[(i - 1) % len(route)]][route[i]] - _distances[route[j]][route[(j + 1) % len(route)]] + _distances[route[(i - 1) % len(route)]][route[(j + 1) % len(route)]]

            # If the distance difference is smaller, swap the cities
            if distance_difference < 0:
                route[i], route[j] = route[j], route[i]

    return tuple(route)
