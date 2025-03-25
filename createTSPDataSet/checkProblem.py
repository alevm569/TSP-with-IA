import numpy as np
import datetime as dt

from sklearn.manifold import MDS

from TSP import report_time
from TSPSolution import TSPSource
from TSP_LP.TSP_LP_Generate import generate_solution_with_heuristics, get_edges_from_solution, get_solution_with_lp
from utils.constants import Heuristics
from utils.plotUtil import plot_route

show_plot = True
show_name = True
matrix_distances = np.array([
    [0, 10, 15, 20],
    [10, 0, 35, 25],
    [15, 35, 0, 30],
    [20, 25, 30, 0],
])

# from distance create cities as  Cities = dict[str, tuple[float, float]]
# Crear el modelo MDS
mds = MDS(n_components=2, dissimilarity='precomputed', random_state=123)

# Ajustar el modelo y transformar las distancias en coordenadas
coordinates = mds.fit_transform(matrix_distances)

# Asignar la primera ciudad a la coordenada (0,0)
coordinates -= coordinates[0]

# Crear el diccionario de ciudades con sus coordenadas
cities = {str(i): (coordinates[i, 0], coordinates[i, 1]) for i in range(len(coordinates))}

distances = {}
# get distances as Distances = dict[str, dict[str, float]]
for i in range(len(matrix_distances)):
    for j in range(len(matrix_distances[i])):
        if i != j:
            from_to = (f"{i}", f"{j}")
            distances[from_to] = np.sqrt((cities[f"{i}"][0] - cities[f"{j}"][0]) ** 2 + (cities[f"{i}"][1] - cities[f"{j}"][1]) ** 2)


ini_time = dt.datetime.now()
solutions = generate_solution_with_heuristics(cities, distances, seed=123, n_solutions=12, verbose=False)
min_h_solution, max_h_solution, best_edges = get_edges_from_solution(solutions)
min_h_solution.source = TSPSource.NEAREST_NEIGHBOR
if show_plot:
    plot_route(cities, distances, max_h_solution.route, title="Nearest Neighbor + 2-opt Max",
                   show_name=show_name, marked_edges=best_edges)
    plot_route(cities, distances, min_h_solution.route, title="Nearest Neighbor + 2-opt Min",
                   show_name=show_name, marked_edges=best_edges)
    print(f"-Solved with NN and 2-opt. \t| distance: {round(min_h_solution.distance,6)}  \t| {report_time(ini_time, dt.datetime.now())}")

    # Generate a sample using the linear programming model with the best edges and 2-opt algorithm
    ini_time = dt.datetime.now()
    heuristics = [Heuristics.BestEdges, Heuristics.NearestNeighbour]
    lp_solution = get_solution_with_lp(cities, distances, heuristics, min_h_solution, max_h_solution, best_edges,
                                       show_name, show_plot, verbose=False)

    print(lp_solution)