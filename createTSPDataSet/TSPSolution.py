import hashlib
import uuid
from enum import Enum
from typing import List
from createTSPDataSet.utils.constants import Cities, Distances, EdgeList
from createTSPDataSet.utils.distanceUtil import get_matrix_distance_from_distance_dict

class TSPSource(Enum):
    NEAREST_NEIGHBOR = "NEAREST_NEIGHBOR"
    ACO = "ACO"
    LP = "LP"
    NONE = "NONE"



class TSPSolution:
    def __init__(self, cities: Cities, distances,  route: List[str], distance: float):
        self.cities: Cities = cities
        self.distances: Distances = distances
        self.matrix_distances = get_matrix_distance_from_distance_dict(len(cities), cities, distances)
        self.hash_id = uuid.uuid4()
        self.route = route
        self.distance = distance
        self.directed_edges : EdgeList = []
        self.edges : EdgeList = []
        self.create_edges_path()
        self.source: TSPSource = TSPSource.NONE


    def get_hash_id(self):
        cities_keys = list(self.cities.keys())
        cities_keys.sort()
        hash_str = ""
        for k in cities_keys:
            hash_str += str(self.cities[k])
        return hashlib.md5(hash_str.encode("utf-8")).hexdigest()

    def create_edges_path(self):
        self.edges = []
        for i in range(len(self.route) - 1):
            self.edges.append((self.route[i], self.route[i + 1]))
            self.edges.append((self.route[i + 1], self.route[i]))
            self.directed_edges.append((self.route[i], self.route[i + 1]))

    def save_as_pickle(self, folder_path: str):
        import os
        import pickle
        self.hash_id = self.get_hash_id()
        if os.path.exists(folder_path) is False:
            os.makedirs(folder_path)

        file_path = os.path.join(folder_path, f"{self.hash_id}.pkl")

        with open(file_path, 'wb') as f:
            pickle.dump(self, f)

    def plot(self):
        from createTSPDataSet.utils.plotUtil import plot_route
        plot_route(self.cities, self.distances, self.route, title=f"Sample solved with {self.source.name}")
