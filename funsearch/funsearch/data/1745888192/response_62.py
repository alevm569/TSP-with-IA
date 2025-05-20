def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Create an initial route by selecting a random city as the starting point and then iteratively selecting the nearest unvisited city.
    num_cities = len(_distances)
    current_city = np.random.randint(num_cities)
    route = [current_city]

    # Visit each city exactly once.
    visited = set([current_city])

    while len(visited) < num_cities:
        # Find the nearest unvisited city.
        nearest_city = None
        min_distance = float('inf')
        for city in range(num_cities):
            if city not in visited:
                distance = _distances[current_city][city]
                if distance < min_distance:
                    nearest_city = city
                    min_distance = distance

        # Add the nearest city to the route and mark it as visited.
        route.append(nearest_city)
        visited.add(nearest_city)

        # Update the current city.
        current_city = nearest_city

    # Return to the starting city.
    route.append(route[0])

    return tuple(route)
