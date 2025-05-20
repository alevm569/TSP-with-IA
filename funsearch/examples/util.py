import os
import sys
import random
import pickle
from numpy import ndarray
#CHECK PROBLEM EVALUATOR
#from funsearch.funsearch.constants import n_cities_graph
#TSP SPEC - LLM
from funsearch.constants import n_cities_graph

base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'createTSPDataSet'))
sys.path.append(base_path)

from TSPSolution import TSPSolution

def load_pkl_from_test(file_path: str) -> TSPSolution:
    if not os.path.exists(file_path):
        raise FileNotFoundError("No .pkl files found in the target directory.")

    with open(file_path, 'rb') as f:
        data = pickle.load(f)

    return data


def read_distance_matrix() -> ndarray:
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    target_dir = os.path.join(base_dir, 'tests', f'tsp-{n_cities_graph}')
    file_list = os.listdir(target_dir)
    if len(file_list) == 0:
        raise FileNotFoundError("No .pkl files found in the target directory.")
    filename = file_list[1]
    # filename = "280854846b3f43a38f7fcdd2fd7ba335.pkl"
    file_path = os.path.join(target_dir, filename)
    data = load_pkl_from_test(file_path)
    return data.matrix_distances
