def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` with a hybrid heuristic."""

    # Create a set of unvisited cities
    unvisited = set(range(len(_distances)))

    # Start from the first city
    current = 0
    route = [current]

    # While there are still unvisited cities
    while unvisited:
        # Get the distances to unvisited cities from the current city
        distances = _distances[current][list(unvisited)]

        # Select the city with the minimum distance
        next_city = min(unvisited, key=lambda c: distances[c])

        # Add the next city to the route and remove it from the unvisited set
        route.append(next_city)
        unvisited.remove(next_city)

        # Update the current city
        current = next_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
