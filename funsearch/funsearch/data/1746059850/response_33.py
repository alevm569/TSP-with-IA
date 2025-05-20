def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Initialize the route with the first city
    route = [0]

    # Create a set of unvisited cities
    unvisited = set(range(1, len(_distances)))

    # Use the nearest neighbor heuristic to find the next city
    while unvisited:
        current_city = route[-1]
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited.remove(nearest_city)

    # Return to the starting city
    route.append(0)

    return tuple(route)
