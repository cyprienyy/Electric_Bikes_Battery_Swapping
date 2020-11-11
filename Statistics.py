import numpy as np
from matplotlib import pyplot as plt
from collections import Counter
import csv
from Changing_Rules import ChangingRules

station_num = 40


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
    return station_demand


def show_station_inflow():
    demands = np.load('demands.npy')
    demands[:, 4] = np.ceil(demands[:, 4] / 10) * 10
    station_inflow = Counter()
    for i in range(demands.shape[0]):
        station_inflow[demands[i, 1]] += demands[i, -1]
    stations = list(station_inflow.keys())
    inflow = [station_inflow[key] for key in stations]
    plt.bar(stations, inflow)
    plt.show()
    return station_inflow


def show_bikes():
    station_inventory = np.load('station_inventory.npy')
    w_i = np.array([8, 6, 6, 4, 4, 2, 2, 0, 0, 0, 0])
    for i in range(station_num):
        inv = station_inventory[i, :]
        yield sum(inv[0:3]), sum(inv[0:7]), sum(inv), np.dot(w_i, inv)


def write_csv():
    station_demand = show_station_demand()
    station_inflow = show_station_inflow()
    show_bikes_iter = show_bikes()
    f = open('demand and inflow.csv', 'w', newline='')
    csv_writer = csv.writer(f, delimiter=',')

    csv_writer.writerow(["demand", "inflow", 'low-battery', 'change_available', 'total', 'weight'])
    for i in range(1, station_num + 1):
        inv = next(show_bikes_iter)
        csv_writer.writerow([station_demand[i], station_inflow[i], inv[0], inv[1], inv[2], inv[3]])
        print(inv)

    f.close()


def calculate_original_loss():
    station_loss = np.zeros((50, 40))
    for i in range(50):
        changingRules = ChangingRules()
        changingRules.stimulate(3600)
        station_loss[i, :] = np.array(changingRules.calculate_loss())

    with open('station_loss.csv', 'w', newline='') as f:
        csv_writer = csv.writer(f, delimiter=',')
        for i in range(50):
            csv_writer.writerow(station_loss[i, :])
    return


def calculate_loss_1():
    station_loss = np.zeros((50, 40))
    for i in range(50):
        changingRules = ChangingRules()
        changingRules.prepare_route_builder()
        changingRules.get_station_info_by_moment(0)
        changingRules.update_existed_tasks()
        changingRules.produce_tasks()
        changingRules.routeBuilder.build_initial_solution()
        changingRules.get_routed_result(3600)
        changingRules.stimulate(3600)
        station_loss[i, :] = np.array(changingRules.calculate_loss())

    with open('station_loss_1.csv', 'w', newline='') as f:
        csv_writer = csv.writer(f, delimiter=',')
        for i in range(50):
            csv_writer.writerow(station_loss[i, :])
    return


if __name__ == '__main__':
    write_csv()
