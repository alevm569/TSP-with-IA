import os, sys
script_path = os.path.dirname(os.path.realpath(__file__))
create_path = os.path.dirname(script_path)
project_path = os.path.dirname(create_path)
sys.path.append(project_path)

from TSP import data_path
from TSPSolution import TSPSolution


# create a reader for TSP Solution, use the above class to read the solution
def read_solution_from_pickle(file_path: str) -> TSPSolution or None:
    import pickle
    if os.path.exists(file_path) is False:
        return None
    with open(file_path, 'rb') as f:
        tsp_solution = pickle.load(f)
    return tsp_solution

file_name = "486505b28b77049bb319293a91101683.pkl"
file_path = os.path.join(data_path, file_name)

read_tsp_solution = read_solution_from_pickle(file_path)
if read_tsp_solution is not None:
    read_tsp_solution.plot()
    print(read_tsp_solution.source)
    print(read_tsp_solution.distance)