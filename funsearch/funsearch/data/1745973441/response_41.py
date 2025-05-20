def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Create a list of unvisited cities
    unvisited = list(range(len(_distances)))

    # Start from the first city
    current_city = 0

    # Initialize the route
    route = []

    # Visit all cities exactly once
    while unvisited:
        # Find the nearest unvisited city
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])

        # Add the nearest city to the route
        route.append(nearest_city)

        # Remove the nearest city from the list of unvisited cities
        unvisited.remove(nearest_city)

        # Update the current city
        current_city = nearest_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
