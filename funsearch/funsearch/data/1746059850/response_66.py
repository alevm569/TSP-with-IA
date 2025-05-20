def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""

    # Create a list of candidate routes using the nearest neighbor heuristic.
    candidate_routes = funsearch.nearest_neighbor(len(_distances))

    # Evaluate each candidate route using the total distance heuristic.
    candidate_route_distances = [calculate_route_distance(route, _distances) for route in candidate_routes]

    # Find the candidate route with the lowest distance.
    best_route_index = np.argmin(candidate_route_distances)
    best_route = candidate_routes[best_route_index]

    return best_route
