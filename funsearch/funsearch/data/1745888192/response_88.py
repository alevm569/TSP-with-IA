def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement your new heuristic here.
    # You may combine existing heuristics or use entirely new ones.

    # Example heuristic:
    # Use a nearest neighbor algorithm to generate an initial route.
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(len(_distances)))

    while remaining_cities:
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    return tuple(route)
