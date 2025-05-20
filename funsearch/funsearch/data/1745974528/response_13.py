def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Apply the nearest neighbor heuristic to find an initial tour.
    current_city = 0
    tour = [current_city]
    remaining_cities = set(range(1, len(_distances)))

    while remaining_cities:
        # Find the city with the shortest distance from the current city.
        nearest_city = min(remaining_cities, key=lambda city: _distances[current_city][city])
        tour.append(nearest_city)
        remaining_cities.remove(nearest_city)
        current_city = nearest_city

    # Perform local search to improve the tour.
    for i in range(len(tour)):
        for j in range(i + 1, len(tour)):
            # Swap two cities in the tour and calculate the new distance.
            new_distance = _distances[tour[i]][tour[j]] + _distances[tour[(j + 1) % len(tour)]][tour[(i - 1) % len(tour)]] - _distances[tour[i]][tour[(j + 1) % len(tour)]] - _distances[tour[(i - 1) % len(tour)]][tour[j]]

            # If the new distance is shorter, swap the two cities.
            if new_distance < _distances[tour[i]][tour[j]]:
                tour[i], tour[j] = tour[j], tour[i]

    return tuple(tour)
