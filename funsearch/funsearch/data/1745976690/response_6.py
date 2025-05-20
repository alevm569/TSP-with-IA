def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v1` with added heuristic.

    Heuristic:
        - Sort the distances between each city and the starting city in ascending order.
        - Start from the city with the smallest distance to the starting city.
        - Iteratively add the city with the smallest distance to the current route, ensuring it hasn't been visited before.

    """

    # Get the number of cities
    num_cities = len(_distances)

    # Create a list of unvisited cities
    unvisited_cities = list(range(num_cities))

    # Initialize the starting city
    current_city = unvisited_cities[0]

    # Initialize the route
    route = [current_city]

    # Iterate until all cities are visited
    while len(unvisited_cities) > 0:

        # Find the city with the smallest distance to the current city
        min_distance = float('inf')
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

    # Return the route
    return tuple(route)
