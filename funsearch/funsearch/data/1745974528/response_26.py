def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid approach."""

    # Initialize the route using the nearest neighbor heuristic.
    start_city = 0
    current_city = start_city
    route = [current_city]

    # Create a set of unvisited cities.
    unvisited_cities = set(range(len(_distances)))
    unvisited_cities.remove(start_city)

    # Iterate until all cities have been visited.
    while unvisited_cities:
        # Find the city with the shortest distance from the current city.
        min_distance = float('inf')
        next_city = None
        for city in unvisited_cities:
            distance = _distances[current_city][city]
            if distance < min_distance:
                min_distance = distance
                next_city = city

        # Add the next city to the route.
        route.append(next_city)
        unvisited_cities.remove(next_city)

        # Update the current city.
        current_city = next_city

    # Return to the starting city.
    route.append(start_city)

    return tuple(route)
