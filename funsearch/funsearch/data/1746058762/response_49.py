def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initial route using nearest neighbor
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        # Find the closest unvisited city
        closest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        route.append(closest_city)
        remaining_cities.remove(closest_city)
        current_city = closest_city

    # Close the route by returning to the starting city
    route.append(route[0])

    return tuple(route)
