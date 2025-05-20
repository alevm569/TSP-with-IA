def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize the route with the first city
    route = [0]

    # Create a set of unvisited cities
    unvisited = set(range(1, len(_distances)))

    # Repeat until all cities have been visited
    while unvisited:
        # Get the current city
        current_city = route[-1]

        # Find the closest unvisited city
        closest_city = min(unvisited, key=lambda city: _distances[current_city][city])

        # Add the closest city to the route
        route.append(closest_city)

        # Remove the closest city from the set of unvisited cities
        unvisited.remove(closest_city)

    # Return the route
    return tuple(route)
