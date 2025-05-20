def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Perform nearest neighbor search to generate an initial route.
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(len(_distances))) - {current_city}

    while remaining_cities:
        # Find the closest unvisited city.
        best_distance = math.inf
        best_city = None
        for city in remaining_cities:
            distance = _distances[current_city][city]
            if distance < best_distance:
                best_distance = distance
                best_city = city

        # Add the closest city to the route and remove it from the remaining cities.
        route.append(best_city)
        remaining_cities.remove(best_city)
        current_city = best_city

    # Close the route by returning to the starting city.
    route.append(route[0])

    return tuple(route)
