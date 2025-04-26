import os
import sys
import random
import pickle
from numpy import ndarray

from funsearch.constants import n_cities_graph

base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'createTSPDataSet'))
sys.path.append(base_path)

from TSPSolution import TSPSolution
from TSP import data_path


def load_random_pkl_from_test(filename: str, n_cities) -> TSPSolution:
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    target_dir = os.path.join(base_dir, 'tests', f'tsp-{n_cities}')

    file_path = os.path.join(target_dir, filename)
    if not os.path.exists(file_path):
        raise FileNotFoundError("No .pkl files found in the target directory.")
    # pkl_files = [f for f in os.listdir(target_dir) if f.endswith('.pkl')]
    # if not pkl_files:
    #     raise FileNotFoundError("No .pkl files found in the target directory.")
    #
    # random_file = random.choice(pkl_files)
    # file_path = os.path.join(target_dir, random_file)
    # print(f"Selected file to read: {random_file}")

    with open(file_path, 'rb') as f:
        data = pickle.load(f)

    return data


def read_distance_matrix() -> ndarray:
    data = load_random_pkl_from_test("09e5c0155b7d12a82437296f53dd82dd.pkl", n_cities_graph)
    #n_cities 100 archivo 33b12090221d136a3aa3d5e7814bc98d
    return data.matrix_distances
