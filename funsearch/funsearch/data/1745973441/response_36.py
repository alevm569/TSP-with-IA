def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialize variables
    num_cities = len(_distances)
    current_city = 0
    route = [current_city]

    # Generate a set of unvisited cities
    unvisited_cities = set(range(num_cities))
    unvisited_cities.remove(current_city)

    # Iterate until all cities are visited
    while unvisited_cities:
        # Find the closest unvisited city
        min_distance = math.inf
        closest_city = None
        for city in unvisited_cities:
            distance = _distances[current_city][city]
            if distance < min_distance:
                min_distance = distance
                closest_city = city

        # Add the closest city to the route and mark it as visited
        route.append(closest_city)
        unvisited_cities.remove(closest_city)
        current_city = closest_city

    # Return the route by appending the starting city
    route.append(route[0])
    return tuple(route)
