def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using the k-medoids clustering algorithm."""

    # Perform k-medoids clustering to identify potential starting points
    num_cities = len(_distances)
    num_clusters = int(np.sqrt(num_cities))
    medoids = funsearch.kmedoids(_distances, num_clusters)

    # Iterate through potential starting points and find the best route
    best_distance = float('inf')
    best_route = None

    for medoid in medoids:
        route = funsearch.nearest_neighbor(_distances, start_city=medoid)
        distance = calculate_route_distance(route, _distances)

        if distance < best_distance:
            best_distance = distance
            best_route = route

    return best_route
