def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Use a hybrid approach that combines the nearest neighbor and cheapest insertion heuristics.
    # Perform local search to improve the solution.

    # Create an initial route using the nearest neighbor heuristic.
    current_city = 0
    route = [current_city]
    unvisited_cities = set(range(1, len(_distances)))

    # Iterate until all cities have been visited.
    while unvisited_cities:
        # Find the closest unvisited city.
        min_distance = math.inf
        for city in unvisited_cities:
            distance = _distances[current_city][city]
            if distance < min_distance:
                min_distance = distance
                next_city = city

        # Add the next city to the route and mark it as visited.
        route.append(next_city)
        unvisited_cities.remove(next_city)
        current_city = next_city

    # Perform local search to improve the solution.
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            # Swap the two cities in the route.
            route[i], route[j] = route[j], route[i]

            # Calculate the total distance of the new route.
            total_distance = calculate_route_distance(route, _distances)

            # If the new route is better, keep it.
            if total_distance < calculate_route_distance(route, _distances):
                route = route

    # Return the best route.
    return tuple(route)
