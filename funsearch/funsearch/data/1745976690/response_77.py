def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2` using a hybrid heuristic.

    Hybrid Heuristic:
    - Starts with a random initial route.
    - Iteratively improves the route by:
        - Applying the 2-opt heuristic to find the best pair of cities to swap.
        - Using the cheapest insertion heuristic to add the next city to the route.

    Returns:
    A tuple of city indices representing the best route.
    """

    num_cities = len(_distances)

    # Initialize a random route.
    best_route = np.random.permutation(num_cities)

    # Iterate until convergence.
    while True:
        # Apply 2-opt heuristic to find the best pair of cities to swap.
        best_delta = 0
        for i in range(num_cities):
            for j in range(i + 1, num_cities):
                delta = _distances[best_route[i]][best_route[(j - 1) % num_cities]] + \
                       _distances[best_route[j]][best_route[(i + 1) % num_cities]] - \
                       _distances[best_route[i]][best_route[j]]
                if delta < best_delta:
                    best_delta = delta
                    best_i = i
                    best_j = j

        # If no improvement was found, the route is optimal.
        if best_delta == 0:
            break

        # Swap the best pair of cities.
        best_route[best_i], best_route[best_j] = best_route[best_j], best_route[best_i]

        # Use cheapest insertion to add the next city.
        best_city = None
        best_distance = float('inf')
        for city in range(num_cities):
            if city not in best_route:
                distance = _distances[best_route[-1]][city]
                if distance < best_distance:
                    best_distance = distance
                    best_city = city

        # Add the next city to the route.
        best_route = np.append(best_route, best_city)

    # Return the best route.
    return best_route.astype(int)
