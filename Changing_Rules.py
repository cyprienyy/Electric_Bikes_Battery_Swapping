import numpy as np
from Stimulation import *
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
        self.station_info = []

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
            self.station_info.append([time_labels, bikes_dist, loss])

    def prepare_route_builder(self):
        self.routeBuilder = RouteBuilder(_dis_mat, _t_mat)
        self.routeBuilder.add_empty_route([0] * _vehicle_num, [0] * _vehicle_num, [0] * _vehicle_num,
                                          [3600] * _vehicle_num)  # 应该添加一些宏观变量用于控制车辆的起始点，终点，时间上限与时间下限

    def produce_tasks(self, lp, up):
        # 引入ban list
        _tasks = []
        for i, info in enumerate(self.station_info):
            _task = self.rule_1(info, lp, up)
            if _task:
                _tasks.append((i + 1,) + _task)
        _tasks = list(zip(*_tasks))
        self.routeBuilder.add_tasks(_tasks[0], _tasks[1], _tasks[2], _tasks[3], _tasks[4])
        self.routeBuilder.build_initial_solution()
        self.routeBuilder.multiple_neighborhood_search()

    @classmethod
    def rule_1(cls, info, lp, up):
        # rule1在一个站点低于30%电量车达到一定数量后产生时间窗下限，在损失达到一定值后产生上限。
        time_labels, bikes_num_info, loss_info = info
        s_t = None
        e_t = None
        for i, bikes_num in enumerate(bikes_num_info):
            if sum(bikes_num[0:3]) >= 3:
                s_t = time_labels[i]
                de = sum(bikes_num[0:3])
                for j, loss in enumerate(loss_info[i + 1:]):
                    if loss - loss_info[i] >= 2:
                        e_t = time_labels[i + j + 1]
                        break
                if e_t is None:
                    e_t = time_labels[-1]
                break
        ser_t = 180
        if s_t is not None:
            return s_t, e_t, de, ser_t
        else:
            return ()


if __name__ == '__main__':
    changingRules = ChangingRules()
    changingRules.prepare_route_builder()
    changingRules.prepare_event_list()
    changingRules.stimulate()
    changingRules.get_station_info(0, 3600)
    changingRules.produce_tasks(0, 3600)
    print()
