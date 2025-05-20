def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Create a set of available cities.
    available_cities = set(range(len(_distances)))

    # Start with an initial route containing only the starting city.
    current_city = 0
    route = [current_city]

    # Iterate until all cities have been visited.
    while available_cities:
        # Find the city with the shortest distance to the current city.
        min_distance = math.inf
        next_city = None
        for city in available_cities:
            distance = _distances[current_city][city]
            if distance < min_distance:
                min_distance = distance
                next_city = city

        # Add the next city to the route and remove it from the available cities.
        route.append(next_city)
        available_cities.remove(next_city)

        # Update the current city.
        current_city = next_city

    # Return the route, including the return to the starting city.
    return tuple(route)
