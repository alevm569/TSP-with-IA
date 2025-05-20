def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Use nearest neighbor to generate an initial route
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(len(_distances)))
    remaining_cities.remove(current_city)

    while remaining_cities:
        # Find the nearest unvisited city
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Add the return to the starting city
    route.append(route[0])

    return tuple(route)
