# ---- mandatory imports ----
import os
import sys

from create_loggers import tsp_logger

script_path = os.path.dirname(os.path.abspath(__file__))
project_path = os.path.dirname(script_path)
sys.path.append(project_path)
# ---- mandatory imports ----
import datetime as dt
from typing import List
from TSPSolution import TSPSolution, TSPSource
from TSP_ACO.TSP_ACO_Generate import generate_aco_solution
from utils.plotUtil import plot_route
from utils.constants import Heuristics, Cities, Distances
from utils.nUtil import find_nearest_neighbor_path_solution, find_best_route_2opt
from utils.generateUtil import generate_cities_with_distances

data_path = os.path.join(script_path, "data")


def get_best_path_nearest_neighbor_and_2opt(cities: Cities, distances: Distances, seed=123):
    ruta = find_nearest_neighbor_path_solution(cities, distances, seed)
    ruta, distance = find_best_route_2opt(distances, ruta, seed)
    return ruta, distance


def report_time(start_time: dt.datetime, end_time: dt.datetime):
    d_time = end_time - start_time
    seconds = d_time.total_seconds()
    d_formatted = f"{int(seconds // 3600)}h {int((seconds % 3600) // 60)}m {int(seconds % 60)}s"
    return f"{seconds} seconds -> ({d_formatted})"


def generate_sample(n_cities: int, seed=123, show_name=False, show_plot=False):
    from TSP_LP.TSP_LP_Generate import generate_solution_with_heuristics, get_edges_from_solution, \
        get_solution_with_lp

    tsp_logger.info(f"--> Generating sample with {n_cities} cities.")
    cities, distances = generate_cities_with_distances(n_cities, seed)
    # Generate a sample using the nearest neighbor heuristic and 2-opt algorithm
    ini_time = dt.datetime.now()
    solutions = generate_solution_with_heuristics(cities, distances, seed=seed, n_solutions=12, verbose=False)
    min_h_solution, max_h_solution, best_edges = get_edges_from_solution(solutions)
    min_h_solution.source = TSPSource.NEAREST_NEIGHBOR
    if show_plot:
        plot_route(cities, distances, max_h_solution.route, title="Nearest Neighbor + 2-opt Max",
                   show_name=show_name, marked_edges=best_edges)
        plot_route(cities, distances, min_h_solution.route, title="Nearest Neighbor + 2-opt Min",
                   show_name=show_name, marked_edges=best_edges)
    tsp_logger.info(
        f"-Solved with NN and 2-opt. \t| distance: {round(min_h_solution.distance,6)}  \t| {report_time(ini_time, dt.datetime.now())}")

    # Generate a sample using the linear programming model with the best edges and 2-opt algorithm
    ini_time = dt.datetime.now()
    heuristics = [Heuristics.BestEdges, Heuristics.NearestNeighbour]
    lp_solution = get_solution_with_lp(cities, distances, heuristics, min_h_solution, max_h_solution, best_edges,
                                       show_name, show_plot, verbose=False)
    if lp_solution is not None:
        lp_solution.source = TSPSource.LP
        solutions.append(lp_solution)
        tsp_logger.info(
            f"-Solved with LP and 2-opt. \t| distance: {round(lp_solution.distance, 6)} \t| {report_time(ini_time, dt.datetime.now())}")

    # Generate a sample using the ant colony optimization algorithm
    ini_time = dt.datetime.now()
    aco_tsp_solution = generate_aco_solution(cities, distances, show_name, show_plot, verbose=False)
    if aco_tsp_solution is not None:
        aco_tsp_solution.source = TSPSource.ACO
        solutions.append(aco_tsp_solution)
        tsp_logger.info(
            f"-Solved with ACO and 2-opt. \t| distance: {round(aco_tsp_solution.distance, 6)} \t| {report_time(ini_time, dt.datetime.now())}")

    if len(solutions) > 0:
        save_sample(solutions, seed)


def save_sample(solution_list: List[TSPSolution], seed: int):
    best_solution = solution_list[0]
    for solution in solution_list:
        if solution is None:
            continue
        if solution.distance < best_solution.distance:
            best_solution = solution
    best_algorithms = [best_solution.source.name]
    for solution in solution_list:
        if round(solution.distance, 5) == round(best_solution.distance, 5) and solution.source.name !=  TSPSource.NONE.name:
            best_algorithms.append(solution.source.name)
    best_solution.best_algorithm = list(set(best_algorithms))

    tsp_logger.info(f"Best solution for {seed}: {best_solution.best_algorithm} - {round(best_solution.distance, 4)}")
    best_solution.save_as_pickle(data_path)


if __name__ == "__main__":
    n_cities = 20
    for i in range(1, 20):
        generate_sample(n_cities, show_name=True, seed=i, show_plot=False)

