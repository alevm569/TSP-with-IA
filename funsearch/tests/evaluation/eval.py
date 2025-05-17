import os
import sys

from best_program_1746499826_2_20 import best_program_route, calculate_route_distance

script_path = os.path.dirname(os.path.abspath(__file__))
eval_tsp_path = os.path.dirname(script_path)
project_path = os.path.dirname(eval_tsp_path)
sys.path.append(project_path)
sys.path.append(eval_tsp_path)

from eval_tsp.plotUtil import read_solution_from_pickle

script_path = os.path.dirname(os.path.abspath(__file__))
test_path = os.path.dirname(script_path)
folder_path = os.path.join(test_path, "tsp-20")

# get name of files
files = os.listdir(folder_path)
files = files[:2]  # Limit to first 5 files for testing
for file in files:
    print(f"Processing file: {file}")
    file_path = os.path.join(folder_path, file)
    tsp_solution = read_solution_from_pickle(file_path)
    matrix = tsp_solution.matrix_distances
    found_route = best_program_route(matrix)
    distance_best_route = calculate_route_distance(found_route, matrix)
    print(f"Best distance from best program: {distance_best_route}")
    print(f"Distance evaluator: {tsp_solution.distance}")
    tsp_solution.plot()
    # plot the best program route
    from eval_tsp.plotUtil import plot_route
    found_route = [str(city) for city in found_route]
    found_route.append(found_route[0])  # Close the loop
    plot_route(tsp_solution.cities, tsp_solution.distances, found_route, title=f"Best Program Route")


