def find_best_route_vx(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Apply a combination of heuristics:
    # 1. Nearest neighbor to generate an initial tour.
    # 2. Cheapest insertion to refine the tour.
    # 3. 2-opt local search to optimize the tour.

    # Generate an initial tour using nearest neighbor
    start_city = 0
    tour = [start_city]
    unvisited = set(range(1, len(_distances)))

    while unvisited:
        current_city = tour[-1]
        nearest_city = min(unvisited, key=lambda city: _distances[current_city][city])
        tour.append(nearest_city)
        unvisited.remove(nearest_city)

    # Refine the tour using cheapest insertion
    while True:
        best_distance = float('inf')
        for i in range(len(tour)):
            for j in range(i + 1, len(tour)):
                distance = _distances[tour[i]][tour[j]]
                if distance < best_distance:
                    best_distance = distance
                    best_indices = (i, j)

        if best_distance == float('inf'):
            break

        tour.insert(best_indices[0] + 1, tour.pop(best_indices[1]))

    # Optimize the tour using 2-opt local search
    def two_opt(tour):
        best_distance = calculate_route_distance(tour, _distances)
        for i in range(len(tour)):
            for j in range(i + 2, len(tour)):
                new_tour = tour[:i] + tour[j:i:-1] + tour[j + 1:]
                new_distance = calculate_route_distance(new_tour, _distances)
                if new_distance < best_distance:
                    best_distance = new_distance
                    best_tour = new_tour
        return best_tour

    for _ in range(10):
        tour = two_opt(tour)

    return tour
