def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid approach."""

    # Create a set of unvisited cities.
    unvisited_cities = set(range(len(_distances)))

    # Start from the first city.
    current_city = 0

    # Initialize the route.
    route = [current_city]

    # Repeat until all cities are visited.
    while unvisited_cities:
        # Find the nearest unvisited city.
        nearest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])

        # Add the nearest city to the route.
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

        # Update the current city.
        current_city = nearest_city

    # Return to the starting city.
    route.append(route[0])

    return tuple(route)
