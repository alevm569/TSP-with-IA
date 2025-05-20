def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid approach."""

    # Initialize a starting route using the nearest neighbor heuristic.
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(len(_distances))) - {current_city}

    while remaining_cities:
        # Find the city that is closest to the current city but not already in the route.
        closest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(closest_city)
        remaining_cities.remove(closest_city)
        current_city = closest_city

    # Perform local search to improve the route.
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            # Swap two cities in the route and calculate the distance difference.
            distance_difference = _distances[route[i]][route[j]] - _distances[route[(i - 1) % len(route)]][route[i]] - _distances[route[j]][route[(j - 1) % len(route)]] + _distances[route[(i - 1) % len(route)]][route[j]] + _distances[route[i]][route[(j - 1) % len(route)]]

            # If the distance difference is negative, swap the two cities.
            if distance_difference < 0:
                route[i], route[j] = route[j], route[i]

    return tuple(route)
