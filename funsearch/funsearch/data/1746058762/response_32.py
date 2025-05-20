def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Use nearest neighbor to generate an initial solution.
    start_city = 0
    current_city = start_city
    route = [current_city]

    while len(route) < len(_distances):
        closest_city = None
        min_distance = float('inf')

        for city in range(len(_distances)):
            if city not in route:
                distance = _distances[current_city][city]
                if distance < min_distance:
                    min_distance = distance
                    closest_city = city

        current_city = closest_city
        route.append(current_city)

    # Add the return trip to the starting city.
    route.append(start_city)

    return tuple(route)
