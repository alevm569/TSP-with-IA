def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Generate a random initial route
    num_cities = len(_distances)
    initial_route = np.random.permutation(num_cities)

    # Use a local search algorithm to refine the route
    def local_search(route):
        best_route = route
        best_distance = calculate_route_distance(route, _distances)

        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                # Swap two cities in the route
                new_route = route.copy()
                new_route[i], new_route[j] = new_route[j], new_route[i]

                # Calculate the distance of the new route
                new_distance = calculate_route_distance(new_route, _distances)

                # If the new route is better, update the best route
                if new_distance < best_distance:
                    best_route = new_route
                    best_distance = new_distance

        return best_route, best_distance

    # Run the local search algorithm until convergence
    best_route, best_distance = local_search(initial_route)
    while True:
        new_route, new_distance = local_search(best_route)
        if new_distance == best_distance:
            break
        else:
            best_route, best_distance = new_route, new_distance

    return best_route
