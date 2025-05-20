def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Apply the nearest neighbor heuristic to generate an initial route.
    start_city = 0
    route = [start_city]
    unvisited_cities = set(range(1, len(_distances)))

    while unvisited_cities:
        current_city = route[-1]
        nearest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # Add the return trip to the starting city.
    route.append(start_city)

    return tuple(route)
