def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Initialization
    num_cities = len(_distances)
    best_route = tuple(range(num_cities))
    best_distance = calculate_route_distance(best_route, _distances)

    # Hybrid heuristic
    for _ in range(100):  # Number of iterations
        # Randomly swap two cities in the route
        city1, city2 = np.random.randint(num_cities, size=2)
        new_route = list(best_route)
        new_route[city1], new_route[city2] = new_route[city2], new_route[city1]
        new_distance = calculate_route_distance(tuple(new_route), _distances)

        # Update best route if necessary
        if new_distance < best_distance:
            best_distance = new_distance
            best_route = tuple(new_route)

    return best_route
