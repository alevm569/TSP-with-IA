def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Create a set of unvisited cities.
    unvisited = set(range(len(_distances)))

    # Start at the first city.
    current = 0

    # Initialize the route.
    route = [current]

    # Visit all cities.
    while unvisited:
        # Find the closest unvisited city.
        closest = min(unvisited, key=lambda city: _distances[current][city])

        # Add the closest city to the route.
        route.append(closest)

        # Remove the closest city from the set of unvisited cities.
        unvisited.remove(closest)

        # Update the current city.
        current = closest

    # Return to the starting city.
    route.append(route[0])

    return tuple(route)
