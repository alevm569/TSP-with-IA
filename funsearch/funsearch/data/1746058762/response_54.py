def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Create a list of available cities
    available_cities = list(range(len(_distances)))

    # Start with the first city
    current_city = available_cities[0]
    route = [current_city]

    # Iterate until all cities have been visited
    while len(available_cities) > 1:
        # Remove the current city from the list of available cities
        available_cities.remove(current_city)

        # Find the city that is closest to the current city and not already in the route
        best_city = None
        min_distance = math.inf
        for city in available_cities:
            distance = _distances[current_city][city]
            if distance < min_distance:
                best_city = city
                min_distance = distance

        # Add the best city to the route
        route.append(best_city)
        current_city = best_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
