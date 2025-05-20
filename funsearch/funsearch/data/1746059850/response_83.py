def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize a list of unvisited cities
    unvisited = list(range(len(_distances)))

    # Start from the first city
    current = 0
    route = [current]

    # Iterate until all cities are visited
    while len(unvisited) > 0:

        # Find the closest unvisited city
        closest = min(unvisited, key=lambda city: _distances[current][city])

        # Add the closest city to the route
        route.append(closest)

        # Remove the closest city from the list of unvisited cities
        unvisited.remove(closest)

        # Update the current city
        current = closest

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
