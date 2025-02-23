import os, sys

script_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(script_path, "data")
sys.path.append(data_path)


# create a reader for TSP Solution, use the above class to read the solution
def read_solution_from_pickle(file_path: str) -> None:
    import pickle
    if os.path.exists(file_path) is False:
        return None
    with open(file_path, 'rb') as f:
        tsp_solution = pickle.load(f)
        print("tsp_solution",tsp_solution)
    return tsp_solution


# file_name = "486505b28b77049bb319293a91101683.pkl"
folder_path = "1740282290"
sandbox_input = "sandbox0"
call_input = "call0"
inputs_folder = "inputs"
input_pickle = "6.pickle"
output_pickle = "output.pickle"
prog_pickle = "prog.pickle"

file_path_call = os.path.join(data_path, folder_path, sandbox_input, call_input)
file_path_input = os.path.join(data_path, folder_path, sandbox_input, inputs_folder)

path_output = os.path.join(file_path_call, output_pickle)
path_prog = os.path.join(file_path_call, prog_pickle)
path_input = os.path.join(file_path_input, input_pickle)

read_solution_from_pickle(path_output)
read_solution_from_pickle(path_prog)
read_solution_from_pickle(path_input)
