import traceback

import numpy as np
import datetime as dt


from TSPSolution import TSPSolution
from TSP_ACO.TSP_ACO import ant_system
from create_loggers import tsp_logger, tsp_detail_logger
from utils.constants import Cities, Distances
from utils.distanceUtil import get_matrix_distance_from_distance_dict
from utils.nUtil import optimize_with_2_opt_util
from utils.plotUtil import plot_route


def generate_aco_solution(cities: Cities, distances: Distances, show_name: bool = False, show_plot: bool = False, verbose: bool = True):
    try:
        return _generate_aco_solution(cities, distances, show_name, show_plot, verbose)
    except Exception as e:
        tb = traceback.format_exc()
        msg = f"There was an error while generating a solution with ACO: {e}"
        tsp_logger.error(msg)
        tsp_detail_logger.error(f"{msg}\n{tb}")
        return None


def _generate_aco_solution(cities: Cities, distances: Distances, show_name: bool = False, show_plot: bool = False, verbose: bool = True):
    # Parameters ACO
    n_cities = len(cities.keys())
    n_ants = n_cities
    n_iterations = 80
    alpha = 1
    beta = 4
    Q = 1

    # Distance matrix using distances variable as numpy matrix
    matrix_distances = get_matrix_distance_from_distance_dict(n_cities, cities, distances)

    # Pheromone matrix
    pheromones = np.ones((n_cities, n_cities)) * 0.2

    # Run the algorithm
    initial_time = dt.datetime.now()
    ants = ant_system(n_cities, n_ants, n_iterations, matrix_distances, pheromones, alpha, beta, Q, rho=0.01)

    min_path_length = np.min([ant.path_length for ant in ants])
    min_ant = [ant for ant in ants if ant.path_length == min_path_length][0]
    final_time = dt.datetime.now()

    delta_time = final_time - initial_time
    # Printing all results
    verbose and print(f'All best ants: time: {delta_time.seconds // 60}m {delta_time.seconds % 60}s')
    for ant in ants:
        verbose and print("Graph path:", ant.path)
        verbose and print("Path length:", ant.path_length)

    verbose and print('\nBest ant')
    verbose and print("Best graph path:", min_ant.path)
    verbose and print("Best path length:", min_path_length)
    index_to_key = list(cities.keys())

    mapped_path = [str(index_to_key[i]) for i in min_ant.path]
    best_path, distance = optimize_with_2_opt_util(mapped_path, cities, distances)
    if show_plot:
        plot_route(cities, distances, best_path, title="ACO solution", show_name=show_name, marked_edges=None)
    tsp_solution = TSPSolution(cities, distances, best_path, distance)
    return tsp_solution