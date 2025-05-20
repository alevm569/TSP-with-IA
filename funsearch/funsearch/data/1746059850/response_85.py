def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Use the nearest neighbor heuristic to initialize a partial route
    current_city = 0
    route = [current_city]

    # Create a set of unvisited cities
    unvisited = set(range(len(_distances)))
    unvisited.remove(current_city)

    # Iterate until all cities have been visited
    while unvisited:
        # Find the nearest unvisited city
        nearest_city = min(unvisited, key=lambda c: _distances[current_city][c])
        route.append(nearest_city)
        unvisited.remove(nearest_city)
        current_city = nearest_city

    # Add the starting city to the end of the route
    route.append(route[0])

    return tuple(route)
