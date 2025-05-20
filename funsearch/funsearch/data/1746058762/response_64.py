def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize the route with the first city
    route = [0]

    # Create a set of unvisited cities
    unvisited = set(range(1, len(_distances)))

    # Iteratively find the closest unvisited city and add it to the route
    while unvisited:
        current_city = route[-1]
        closest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        route.append(closest_city)
        unvisited.remove(closest_city)

    # Return to the starting city
    route.append(0)

    return tuple(route)
