def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a hybrid heuristic that combines the nearest neighbor and cheapest insertion algorithms.
    # Use a local search algorithm to refine the solution.

    # Example implementation using nearest neighbor and cheapest insertion:
    route = np.random.permutation(len(_distances))
    while True:
        current_city = route[-1]
        nearest_city = np.argmin(_distances[current_city])
        cheapest_city = np.argmin(_distances[current_city][route])
        if nearest_city not in route:
            route = np.append(route, nearest_city)
        elif cheapest_city not in route:
            route = np.append(route, cheapest_city)
        else:
            break

    # Perform local search to refine the route.
    # ...

    return route
