def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Perform nearest neighbor search to find an initial route.
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        # Find the closest unvisited city.
        closest_city = min(remaining_cities, key=lambda c: _distances[current_city][c])
        remaining_cities.remove(closest_city)
        route.append(closest_city)

        # Update the current city.
        current_city = closest_city

    # Close the route by returning to the starting city.
    route.append(route[0])

    return tuple(route)
