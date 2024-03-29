from Stimulation import Station, EventList, BATTERY_LEVEL, DemandEvent, ChargeEvent
from Routes import RouteBuilder, RouteBuilderByDistance
import numpy as np
from Read_Files import resolve_station_inventory
import csv
import re

# 在这里设置好_dis_mat, _t_mat, _capacity, _vehicle_num
file_path = r'.\dm.npy'
_dis_mat = np.load(file_path)
_dis_mat = _dis_mat.astype(int)
_dis_mat = _dis_mat[:41, :41]
_t_mat = _dis_mat / 300 * 60
_t_mat = np.around(_t_mat, 0).astype(int)
_t_mat = _t_mat[:41, :41]
_vehicle_num = 2
_vehicle_capacity = 200
station_num = 40
w_i = np.array([8, 6, 6, 4, 4, 2, 2, 0, 0, 0, 0])


class ChangingRules:
    def __init__(self):
        resolve_station_inventory_iter = resolve_station_inventory()
        self.stations = [Station(next(resolve_station_inventory_iter)) for _ in range(station_num)]

        self.eventList = EventList()
        demands = np.load('demands.npy')
        demands[:, 4] = np.ceil(demands[:, 4] / 10) * 10
        demands = demands.tolist()
        for d in demands:
            self.eventList.add_event(DemandEvent(d[2], d[0], d[3], d[1], d[4], d[5]))

        self.charge_schedule = []
        self.routeBuilder = None
        self.station_info = []
        self.banned_stations = []

    def stimulate(self, lp):
        '''
        for station in self.stations:
            station.clear_state()
        '''
        while self.eventList.next_event() is not None:
            next_event = self.eventList.next_event()
            if next_event.startTime > lp:
                break
            else:
                self.stations[next_event.startPosition - 1].process_event(next_event, self.eventList)
                self.eventList.remove_event()

    '''
    def get_station_info_by_interval(self, lp, up):
        self.station_info = []
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
    '''

    def get_station_info_by_moment(self, current_time):
        self.station_info = []
        for station in self.stations:
            time_labels = list(station.bikesRecord.keys())
            time_labels.sort()
            t_label = current_time
            for _, t in enumerate(time_labels):
                if t <= current_time:
                    t_label = t
                else:
                    break
            bikes_dist = [station.bikesRecord[t_label][bl * 10] for bl in range(11)]
            loss = station.loss[t_label]
            self.station_info.append([[current_time], [bikes_dist], [loss]])

    def calculate_scheduled_demand(self):
        res = 0
        for sch in self.charge_schedule:
            res = res + sch[-1]
        return res

    def calculate_excess_demand(self):
        excess_demand = 0
        for station in self.stations:
            excess_demand = excess_demand + sum(station.onsiteVehicles.values())
        return excess_demand

    def show_bike_distribution(self):
        res = {}
        for bl in BATTERY_LEVEL:
            res[bl] = 0
        for info in self.station_info:
            for bl in BATTERY_LEVEL:
                res[bl] = res[bl] + info[1][0][bl // 10]
        full_battery = 0
        for bl in res.keys():
            if bl <= 60:
                full_battery = full_battery + res[bl]
        return res, full_battery

    def prepare_route_builder(self):
        self.routeBuilder = RouteBuilder(_dis_mat, _t_mat)
        self.routeBuilder.add_empty_route([0] * _vehicle_num, [0] * _vehicle_num, [0] * _vehicle_num,
                                          [3600] * _vehicle_num,
                                          [200] * _vehicle_num)  # 应该添加一些宏观变量用于控制车辆的起始点，终点，时间上限与时间下限

    def produce_tasks(self, *args):
        _tasks = []
        for i, info in enumerate(self.station_info):
            if i in self.banned_stations:
                continue
            _task = self.rule_2(info)
            if _task:
                self.stations[i].predict_value.append(_task[2])
                _tasks.append((i + 1,) + _task + (3600, 0))
        _tasks = sorted(_tasks, key=lambda x: x[5], reverse=True)
        _tasks = _tasks[0:30]

        with open('tasks.csv', 'w', newline='') as f:
            csv_writer = csv.writer(f, delimiter=',')
            csv_writer.writerows(_tasks)

        _tasks = list(zip(*_tasks))
        # location, start_time, end_time, task_demand, service_time, w_i
        if _tasks:
            self.routeBuilder.add_tasks(_tasks[0], _tasks[1], _tasks[2], _tasks[3], _tasks[4], _tasks[5], _tasks[6],
                                        _tasks[7])

    def get_routed_result(self, lp):
        _res = self.routeBuilder.fix_sol(lp)
        self.charge_schedule.extend(_res)
        for c in _res:
            self.eventList.add_event(ChargeEvent(c[1], c[0], c[2], c[3]))
        return

    def get_tasks_from_csv(self):

        with open('newtask.csv', 'r') as file_to_read:
            _tasks = []
            while True:
                lines = file_to_read.readline()  # 整行读取数据
                if not lines:
                    break
                    pass
                if re.match(r'\s*[0-9]', lines) is not None:
                    lines = lines.strip()
                    lines = lines.split(',')
                    lines = list(map(int, lines))
                    _tasks.append(lines)  # 添加新读取的数据
                pass
        pass

        _tasks = list(zip(*_tasks))
        if _tasks:
            self.routeBuilder.add_tasks(_tasks[0], _tasks[1], _tasks[2], _tasks[3], _tasks[4], _tasks[5], _tasks[6],
                                        [tk * 20 for tk in _tasks[7]])

    def get_banned_stations(self):
        self.banned_stations = []
        # print(self.routeBuilder.activeKeys)
        for key in list(self.routeBuilder.activeKeys):
            self.banned_stations.append(self.routeBuilder.trans_key_to_station(key) - 1)
        return

    def update_existed_tasks(self, *args):
        self.get_banned_stations()
        for key in self.routeBuilder.activeKeys:
            info = self.station_info[self.routeBuilder.trans_key_to_station(key) - 1]
            new_task_demand = self.rule_3(info)
            self.stations[self.routeBuilder.trans_key_to_station(key) - 1].predict_value.append(new_task_demand)
            new_w_i = self.get_station_weight_1(info[1][0])
            self.routeBuilder.key2Task[key].demand = new_task_demand
            self.routeBuilder.key2Task[key].w_i = new_w_i
        return

    def update_vehicle_load(self, current_time):
        for i, route in enumerate(self.routeBuilder.fixedRoutes):
            # print('路径',i+1,'原load',self.routeBuilder.routes_info[i]['current_load'])
            load = 0
            for j in route:
                station_j = self.routeBuilder.trans_key_to_station(j)
                if station_j == 0:
                    load = 0
                else:
                    load += self.routeBuilder.key2Task[j].demand
                    leave_time = list(self.stations[station_j - 1].onsiteVehicles.keys())
                    if leave_time:
                        if leave_time[0] < current_time:
                            load -= self.stations[station_j - 1].onsiteVehicles[leave_time[0]]
            self.routeBuilder.routes_info[i]['current_load'] = load
            # print('路径', i + 1, '新load', self.routeBuilder.routes_info[i]['current_load'])
        return

    '''
    @classmethod
    def rule_1(cls, info):
        # rule1在一个站点低于30%电量车达到一定数量后产生时间窗下限，在损失达到一定值后产生上限。
        time_labels, bikes_num_info, loss_info = info
        s_t = None
        e_t = None
        de = None
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
    '''

    @classmethod
    def rule_2(cls, info):
        # rule2用于产生换电任务
        time_label, bikes_num_info, _ = info
        if sum(bikes_num_info[0][0:7]) >= 0:
            return time_label[0], time_label[0] + 3600, sum(bikes_num_info[0][0:7]), 120, cls.get_station_weight_1(
                bikes_num_info[0])
        else:
            return None

    @classmethod
    def get_station_weight_1(cls, bikes_num_info):
        return np.dot(np.array(bikes_num_info), w_i) * 2

    @classmethod
    def get_station_weight_2(cls, bikes_num_info, demand, inflow):
        return min(max((demand - inflow) - sum(bikes_num_info[4:]), 0), sum(bikes_num_info[0:3]))

    @classmethod
    def rule_3(cls, info):
        # rule3用于更新换电的demand
        _, bikes_num_info, _ = info
        return sum(bikes_num_info[0][0:7])

    def calculate_loss(self):
        station_loss = []
        for station in self.stations:
            loss = 0
            for time_label in station.loss.keys():
                loss = loss + station.loss[time_label]
            station_loss.append(loss)
        print(station_loss)
        return station_loss


def static_method():
    changingRules = ChangingRules()
    changingRules.prepare_route_builder()
    changingRules.get_station_info_by_moment(0)
    changingRules.update_existed_tasks()
    changingRules.produce_tasks()
    changingRules.routeBuilder.build_initial_solution()
    print('路径成本',
          changingRules.routeBuilder.evaluate_solution_by_total_distance(changingRules.routeBuilder.best_feas_sol))
    print('总成本', changingRules.routeBuilder.best_feas_obj)
    changingRules.routeBuilder.multiple_neighborhood_search()
    print('路径成本',
          changingRules.routeBuilder.evaluate_solution_by_total_distance(changingRules.routeBuilder.best_feas_sol))
    print('总成本', changingRules.routeBuilder.best_feas_obj)
    changingRules.routeBuilder.print_sol()
    changingRules.get_routed_result(3600)
    changingRules.stimulate(3600)
    changingRules.get_station_info_by_moment(3600)
    print('未分配任务', changingRules.routeBuilder.unassigned_tasks)
    print('总损失', sum(changingRules.calculate_loss()))
    print('分配的多余换电量', changingRules.calculate_excess_demand())
    print('分配的总换电量', changingRules.calculate_scheduled_demand())
    print('换电后车辆情况', changingRules.show_bike_distribution())
    return


def dynamic_method():
    changingRules = ChangingRules()
    changingRules.prepare_route_builder()
    _current_time = 0
    _anticipation_horizon = 600
    _num_time_slices = 6
    for _ts in range(_num_time_slices):
        _current_time = _ts * _anticipation_horizon
        # 获取当前时刻的状态
        changingRules.stimulate(_current_time)

        changingRules.get_station_info_by_moment(_current_time)
        changingRules.update_existed_tasks()
        changingRules.update_vehicle_load(_current_time)
        if _current_time == 0:
            changingRules.produce_tasks()

        changingRules.routeBuilder.build_initial_solution()
        changingRules.routeBuilder.multiple_neighborhood_search()
        # 发送这个时刻得到的，截止到下一个anticipation horizon之前的计划
        # 同时让routeBuilder的状态更新到下一个anticipation horizon开始前的状态
        changingRules.get_routed_result(_current_time + _anticipation_horizon)
    changingRules.stimulate(_num_time_slices * _anticipation_horizon)
    changingRules.get_station_info_by_moment(_num_time_slices * _anticipation_horizon)
    print(sum(changingRules.calculate_loss()))
    print(changingRules.routeBuilder.evaluate_solution_by_total_distance(
        [changingRules.routeBuilder.fixedRoutes[0] + [0], changingRules.routeBuilder.fixedRoutes[1] + [0]]))
    print('未分配任务', changingRules.routeBuilder.unassigned_tasks)
    print(changingRules.calculate_excess_demand())
    print(changingRules.calculate_scheduled_demand())
    print(changingRules.show_bike_distribution())
    print(len(changingRules.charge_schedule))
    print([changingRules.routeBuilder.fixedRoutes[0] + [0], changingRules.routeBuilder.fixedRoutes[1] + [0]])
    return


if __name__ == '__main__':
    static_method()
    for i in range(10):
        dynamic_method()
    print('-----------------')
    print('finished')
