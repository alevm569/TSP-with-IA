def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid approach."""

    # Initialize a set of unvisited cities and a list to store the route.
    unvisited = set(range(len(_distances)))
    route = []

    # Start from the first city.
    current = 0

    # Iterate until all cities are visited.
    while unvisited:
        # Find the closest unvisited city.
        nearest = min(unvisited, key=lambda city: _distances[current][city])

        # Add the nearest city to the route.
        route.append(nearest)
        unvisited.remove(nearest)

        # Update the current city.
        current = nearest

    # Return to the starting city.
    route.append(0)

    return tuple(route)
