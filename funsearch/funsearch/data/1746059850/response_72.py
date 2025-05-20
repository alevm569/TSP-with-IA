def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Initialize a list of unvisited cities
    unvisited_cities = list(range(len(_distances)))

    # Start from the first city
    current_city = 0

    # Initialize the route
    route = [current_city]

    # Visit all cities exactly once
    while unvisited_cities:
        # Find the city with the minimum distance from the current city
        nearest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])

        # Add the nearest city to the route
        route.append(nearest_city)

        # Remove the nearest city from the list of unvisited cities
        unvisited_cities.remove(nearest_city)

        # Update the current city
        current_city = nearest_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
