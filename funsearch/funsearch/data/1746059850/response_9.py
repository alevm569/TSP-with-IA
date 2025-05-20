def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Apply the nearest neighbor heuristic to generate an initial route
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        next_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        route.append(next_city)
        remaining_cities.remove(next_city)
        current_city = next_city

    # Add the return to the starting city
    route.append(route[0])

    return tuple(route)
