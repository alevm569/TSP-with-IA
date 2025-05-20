def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize a set of unvisited cities.
    unvisited = set(range(len(_distances)))

    # Start from the first city.
    current_city = 0
    route = [current_city]

    # While all cities have not been visited, continue the route.
    while unvisited:
        # Find the closest unvisited city.
        closest_city = min(unvisited, key=lambda city: _distances[current_city][city])

        # Add the closest city to the route.
        route.append(closest_city)

        # Remove the closest city from the set of unvisited cities.
        unvisited.remove(closest_city)

        # Update the current city.
        current_city = closest_city

    # Return the route, including the return to the starting city.
    return tuple(route)
