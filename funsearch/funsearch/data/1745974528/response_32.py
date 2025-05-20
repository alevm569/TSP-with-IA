def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Initialize the route with the starting city
    route = [0]

    # Initialize the set of unvisited cities
    unvisited = set(range(1, len(_distances)))

    # Repeat until all cities have been visited
    while unvisited:
        # Get the last city in the route
        current_city = route[-1]

        # Find the closest unvisited city
        best_city = min(unvisited, key=lambda city: _distances[current_city][city])

        # Add the closest city to the route and remove it from the set of unvisited cities
        route.append(best_city)
        unvisited.remove(best_city)

    # Return to the starting city
    route.append(0)

    return tuple(route)
