def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Use the nearest neighbor heuristic to find an initial route.
    current_city = 0
    route = [current_city]
    remaining_cities = set(range(len(_distances))) - {current_city}

    while remaining_cities:
        # Find the city with the shortest distance to the current city.
        min_distance = math.inf
        next_city = None
        for city in remaining_cities:
            distance = _distances[current_city][city]
            if distance < min_distance:
                min_distance = distance
                next_city = city

        # Add the next city to the route and remove it from the remaining cities.
        route.append(next_city)
        remaining_cities.remove(next_city)

        # Update the current city.
        current_city = next_city

    # Close the route by returning to the starting city.
    route.append(route[0])

    return tuple(route)
