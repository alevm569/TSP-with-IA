def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    num_cities = len(_distances)

    # Initialize a list of unvisited cities
    unvisited_cities = list(range(num_cities))

    # Start from the first city
    current_city = 0
    route = [current_city]

    # Continue until all cities have been visited
    while len(unvisited_cities) > 0:
        # Find the closest unvisited city to the current city
        min_distance = math.inf
        next_city = None
        for city in unvisited_cities:
            distance = _distances[current_city][city]
            if distance < min_distance:
                min_distance = distance
                next_city = city

        # Add the next city to the route and mark it as visited
        route.append(next_city)
        unvisited_cities.remove(next_city)

        # Update the current city
        current_city = next_city

    # Return to the starting city
    route.append(route[0])

    return tuple(route)
