def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Create a list of cities
    cities = list(range(len(_distances)))

    # Initialize the best route and distance
    best_route = None
    best_distance = float('inf')

    # Perform a local search
    for _ in range(100):
        # Randomly swap two cities in the route
        city1, city2 = np.random.randint(0, len(cities), 2)
        cities[city1], cities[city2] = cities[city2], cities[city1]

        # Calculate the distance of the new route
        distance = calculate_route_distance(cities, _distances)

        # If the new route is better, update the best route and distance
        if distance < best_distance:
            best_distance = distance
            best_route = cities.copy()

    # Return the best route
    return best_route
