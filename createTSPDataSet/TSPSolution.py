from typing import List
from createTSPDataSet.utils.constants import Cities, Distances, EdgeList


class TSPSolution:
    def __init__(self,cities: Cities, distances,  route: List[str], distance: float):
        self.cities: Cities = cities
        self.distances: Distances = distances
        self.route = route
        self.distance = distance
        self.directed_edges : EdgeList = []
        self.edges : EdgeList = []
        self.create_edges_path()
        self.hash_id = self.get_hash_id()


    def get_hash_id(self):
        return hash(str(self.cities))

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