import numpy as np
from matplotlib import pyplot as plt
from collections import Counter


def show_station_demand():
    demands = np.load('demands.npy')
    demands[:, 4] = np.ceil(demands[:, 4] / 10) * 10
    station_demand = Counter()
    for i in range(demands.shape[0]):
        station_demand[demands[i, 0]] += demands[i, -1]
    stations = list(station_demand.keys())
    demand = [station_demand[key] for key in stations]
    plt.bar(stations, demand)
    plt.show()
