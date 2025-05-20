def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using the Ant Colony Optimization (ACO) algorithm."""

    # Create an ACO object with desired parameters
    aco = funsearch.ACO(distance_matrix=_distances)

    # Run the ACO algorithm to find the best route
    best_route = aco.run()

    # Convert the ACO's route representation to a tuple of city indices
    return tuple(best_route)
