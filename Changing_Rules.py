from Stimulation import *
import numpy as np
from Routes import Task, RouteBuilder
from Read_Files import read_single_soloman, resolve_soloman

# 在这里设置好_dis_mat, _t_mat, _capacity, _vehicle_num
file_path = r'.\solomon_25\C102.txt'
_info, _mat = read_single_soloman(file_path)
_vehicle_num, _capacity, _dis_mat, _, _, _ = resolve_soloman(_info, _mat)
_dis_mat = np.around(_dis_mat, 1)
_t_mat = _dis_mat
_vehicle_num = 3


class ChangingRules:
    def __init__(self):
        self.eventList = EventList()
        self.stations = [Station([20] * 5) for _ in range(10)]
        demands = np.load('demands.npy')
        demands[:, 4] = np.ceil(demands[:, 4] / 10) * 10
        self.demands = demands.tolist()
        self.charge_schedule = []
        self.routeBuilder = None

    def prepare_event_list(self):
        for d in self.demands:
            self.eventList.add_event(DemandEvent(d[2], d[0], d[3], d[1], d[4], d[5]))
        for c in self.charge_schedule:
            self.eventList.add_event(ChargeEvent(c[1], c[0], c[2], c[3]))

    def stimulate(self):
        while self.eventList.next_event() is not None:
            next_event = self.eventList.next_event()
            self.stations[next_event.startPosition - 1].process_event(next_event, self.eventList)
            self.eventList.remove_event()

    def get_station_info(self, lp, up):
        station_info = []
        for station in self.stations:
            time_labels = list(station.bikesRecord.keys())
            time_labels.sort()
            l_index = 0
            u_index = len(time_labels)
            for i, t in enumerate(time_labels):
                if t <= lp:
                    l_index = i
                if t <= up:
                    u_index = i
            time_labels = time_labels[l_index:u_index + 1]
            if time_labels[-1] < up:
                time_labels.append(time_labels[-1])
            bikes_dist = [[station.bikesRecord[t][bl * 10] for bl in range(11)] for t in time_labels]
            loss = [station.loss[t] for t in time_labels]
            if time_labels[0] < lp:
                time_labels[0] = lp
            if time_labels[-1] < up:
                time_labels[-1] = up
            station_info.append([time_labels, bikes_dist, loss])
        return station_info

    def prepare_route_builder(self):
        self.routeBuilder = RouteBuilder(_dis_mat, _t_mat)
        self.routeBuilder.add_empty_route([0] * _vehicle_num, [0] * _vehicle_num, [0] * _vehicle_num,
                                          3600 * _vehicle_num)  # 应该添加一些宏观变量用于控制车辆的起始点，终点，时间上限与时间下限

    def produce_tasks(self, lp, up):
        # 引入ban list
        _tasks = self.rule_1(self.get_station_info(lp, up))
        for d in enumerate(_tasks):
            self.routeBuilder.add_tasks(d[0], d[1], d[2], d[3], d[4])
        self.routeBuilder.build_initial_solution()
        self.routeBuilder.multiple_neighborhood_search()

    def rule_1(self, station_info):
        return []
