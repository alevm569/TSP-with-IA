def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize the route with the first city
    route = [0]

    # Create a set of unvisited cities
    unvisited = set(range(1, len(_distances)))

    # While there are still unvisited cities, find the closest unvisited city to the last city in the route
    while unvisited:
        current_city = route[-1]
        closest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        route.append(closest_city)
        unvisited.remove(closest_city)

    # Return the route, including the return to the starting city
    return tuple(route + [route[0]])
