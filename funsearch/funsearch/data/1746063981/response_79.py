def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or use an existing one with modifications.
    # For example, you could use the nearest neighbor heuristic and refine it with a local search technique.

    # Example using the nearest neighbor heuristic with local search:
    # 1. Find the starting city.
    start_city = 0

    # 2. Create an empty route.
    route = [start_city]

    # 3. Iterate until all cities are visited.
    while len(route) < len(_distances):
        # Find the nearest unvisited city.
        nearest_city = None
        min_distance = float('inf')
        for city in range(len(_distances)):
            if city not in route:
                distance = _distances[route[-1]][city]
                if distance < min_distance:
                    nearest_city = city
                    min_distance = distance

        # Add the nearest city to the route.
        route.append(nearest_city)

    # Perform local search to improve the route.
    for _ in range(10):  # Number of local search iterations
        # Randomly swap two cities in the route.
        i, j = np.random.randint(0, len(route), 2)
        route[i], route[j] = route[j], route[i]

        # Calculate the total distance of the route.
        total_distance = calculate_route_distance(route, _distances)

        # If the new route is better, keep it.
        if total_distance < calculate_route_distance(route, _distances):
            pass
        else:
            # Otherwise, restore the previous route.
            route[i], route[j] = route[j], route[i]

    return tuple(route)
