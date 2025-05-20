def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic here, for example, a hybrid of nearest neighbor and cheapest insertion.

    # Example heuristic using nearest neighbor:
    start_city = 0
    route = [start_city]
    remaining_cities = set(range(len(_distances))) - {start_city}

    while remaining_cities:
        current_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    return tuple(route)
