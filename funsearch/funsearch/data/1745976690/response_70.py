def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""
    # Initialize the route with the starting city
    route = [0]

    # Use the nearest neighbor heuristic to find the next city to visit
    current_city = 0
    while len(route) < len(_distances):
        # Get the list of unvisited cities
        unvisited_cities = list(set(range(len(_distances))) - set(route))

        # Find the nearest unvisited city
        nearest_city = min(unvisited_cities, key=lambda c: _distances[current_city][c])

        # Add the nearest city to the route
        route.append(nearest_city)
        current_city = nearest_city

    # Return to the starting city
    route.append(0)

    return tuple(route)
