def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of find_best_route_v2 using a hybrid heuristic.

    Hybrid heuristic combines the nearest neighbor and cheapest insertion heuristics.
    """

    # Initialize the route with the first city
    route = [0]

    # Create a set of unvisited cities
    unvisited = set(range(1, len(_distances)))

    # Start with the first city and iteratively find the nearest and cheapest unvisited city
    while unvisited:
        # Get the last city in the route
        current_city = route[-1]

        # Find the nearest unvisited city
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])

        # Find the cheapest unvisited city to insert into the route
        cheapest_city = min(unvisited, key=lambda city: calculate_route_distance(route + [city], _distances))

        # Add the nearest and cheapest cities to the route
        route.append(nearest_city)
        route.append(cheapest_city)

        # Remove the visited cities from the set of unvisited cities
        unvisited.remove(nearest_city)
        unvisited.remove(cheapest_city)

    # Return the route
    return tuple(route)
