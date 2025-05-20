def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 using a hybrid heuristic.

    - Starts with the nearest neighbor heuristic.
    - Uses the 2-opt local search algorithm to refine the route.

    Returns:
        A tuple representing the best route.
    """

    # Initialize the route using the nearest neighbor heuristic
    start_city = 0
    route = [start_city]
    unvisited_cities = set(range(len(_distances)))
    unvisited_cities.remove(start_city)

    while unvisited_cities:
        current_city = route[-1]
        nearest_city = min(unvisited_cities, key=lambda city: _distances[current_city][city])
        route.append(nearest_city)
        unvisited_cities.remove(nearest_city)

    # Refine the route using the 2-opt local search algorithm
    for i in range(len(route)):
        for j in range(i + 1, len(route)):
            # Create a copy of the route
            test_route = route[:]

            # Swap the two cities
            test_route[i], test_route[j] = test_route[j], test_route[i]

            # Calculate the distance of the new route
            test_distance = calculate_route_distance(test_route, _distances)

            # If the new route is better, update the current route
            if test_distance < calculate_route_distance(route, _distances):
                route = test_route

    # Return the best route
    return route
