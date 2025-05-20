def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v2`.

    Uses a hybrid heuristic based on the nearest neighbor and 2-opt algorithms.
    """

    # Find the starting city with the lowest distance to any other city.
    start_city = np.argmin(_distances[0])

    # Create an empty route and add the starting city.
    route = [start_city]

    # Visit all other cities.
    remaining_cities = set(range(len(_distances)))
    remaining_cities.remove(start_city)

    while remaining_cities:
        # Find the city with the lowest distance from the last city in the route.
        last_city = route[-1]
        nearest_city = min(remaining_cities, key=lambda c: _distances[last_city][c])

        # Add the nearest city to the route.
        route.append(nearest_city)
        remaining_cities.remove(nearest_city)

    # Perform 2-opt swaps to improve the route.
    for i in range(len(route)):
        for j in range(i + 2, len(route)):
            distance_original = _distances[route[i]][route[j]]
            distance_reversed = _distances[route[i]][route[j - 1]] + _distances[route[j]][route[i]] + _distances[route[j - 1]][route[j]]

            if distance_reversed < distance_original:
                route[i:j] = route[i:j][::-1]

    return tuple(route)
