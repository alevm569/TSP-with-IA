def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1` using a hybrid heuristic."""

    # Create a set to store the unvisited cities.
    unvisited_cities = set(range(len(_distances)))

    # Start from the first city.
    current_city = 0

    # Initialize the route list.
    route = [current_city]

    # Iterate until all cities have been visited.
    while len(unvisited_cities) > 0:
        # Find the city with the shortest distance from the current city.
        min_distance = math.inf
        next_city = None
        for city in unvisited_cities:
            distance = _distances[current_city][city]
            if distance < min_distance:
                min_distance = distance
                next_city = city

        # Add the next city to the route.
        route.append(next_city)

        # Remove the next city from the set of unvisited cities.
        unvisited_cities.remove(next_city)

        # Update the current city.
        current_city = next_city

    # Return to the starting city.
    route.append(route[0])

    return tuple(route)
