import hashlib
from typing import List
from createTSPDataSet.utils.constants import Cities, Distances, EdgeList


class TSPSolution:
    def __init__(self,cities: Cities, distances,  route: List[str], distance: float):
        self.cities: Cities = cities
        self.distances: Distances = distances
        self.hash_id = self.get_hash_id()
        self.route = route
        self.distance = distance
        self.directed_edges : EdgeList = []
        self.edges : EdgeList = []
        self.create_edges_path()


    def get_hash_id(self):
        cities_keys = list(self.cities.keys())
        cities_keys.sort()
        str_cities= "-".join([f"{k}_{self.cities[k]}" for k in cities_keys])
        return hashlib.md5(str_cities.encode("utf-8")).hexdigest()

    def create_edges_path(self):
        self.edges = []
        for i in range(len(self.route) - 1):
            self.edges.append((self.route[i], self.route[i + 1]))
            self.edges.append((self.route[i + 1], self.route[i]))
            self.directed_edges.append((self.route[i], self.route[i + 1]))

    def save_as_pickle(self, folder_path: str):
        import os
        import pickle
        if os.path.exists(folder_path) is False:
            os.makedirs(folder_path)

        file_path = os.path.join(folder_path, f"{self.hash_id}.pkl")

        with open(file_path, 'wb') as f:
            pickle.dump(self, f)

# create a reader for TSP Solution, use the above class to read the solution
def read_solution_from_pickle(file_path: str) -> TSPSolution:
    import pickle
    with open(file_path, 'rb') as f:
        tsp_solution = pickle.load(f)
    return tsp_solution