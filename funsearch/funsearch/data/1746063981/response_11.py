def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize starting point
    start_city = 0
    current_city = start_city
    route = [current_city]

    # Create a set of unvisited cities
    unvisited_cities = set(range(len(_distances)))
    unvisited_cities.remove(start_city)

    # Iterate until all cities are visited
    while unvisited_cities:

        # Use nearest neighbor heuristic to find the closest unvisited city
        closest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])

        # Add the closest city to the route and mark it as visited
        route.append(closest_city)
        unvisited_cities.remove(closest_city)

        # Update the current city
        current_city = closest_city

    # Return to the starting city
    route.append(start_city)

    return tuple(route)
