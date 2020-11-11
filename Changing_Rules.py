from Stimulation import Station, EventList, BATTERY_LEVEL, DemandEvent, ChargeEvent
from Routes import RouteBuilder, RouteBuilderByDistance
import numpy as np
from Read_Files import resolve_station_inventory

# 在这里设置好_dis_mat, _t_mat, _capacity, _vehicle_num
file_path = r'.\dm.npy'
_dis_mat = np.load(file_path)
_dis_mat = _dis_mat.astype(int)
_dis_mat = _dis_mat[:41, :41]
_t_mat = _dis_mat / 250 * 60
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
        return res

    def prepare_route_builder(self):
        self.routeBuilder = RouteBuilderByDistance(_dis_mat, _t_mat)
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
                _tasks.append((i + 1,) + _task)
        _tasks = sorted(_tasks, key=lambda x: x[5], reverse=True)
        _tasks = _tasks[0:30]
        _tasks = list(zip(*_tasks))
        # location, start_time, end_time, task_demand, service_time, w_i
        if _tasks:
            self.routeBuilder.add_tasks(_tasks[0], _tasks[1], _tasks[2], _tasks[3], _tasks[4], _tasks[5])

    def get_routed_result(self, lp):
        _res = self.routeBuilder.fix_sol(lp)
        self.charge_schedule.extend(_res)
        for c in _res:
            self.eventList.add_event(ChargeEvent(c[1], c[0], c[2], c[3]))
        return

    def get_banned_stations(self):
        self.banned_stations = []
        print(self.routeBuilder.activeKeys)
        for key in list(self.routeBuilder.activeKeys):
            self.banned_stations.append(self.routeBuilder.trans_key_to_station(key) - 1)
        return

    def update_existed_tasks(self, *args):
        self.get_banned_stations()
        for key in self.routeBuilder.activeKeys:
            info = self.station_info[self.routeBuilder.trans_key_to_station(key) - 1]
            new_task_demand = self.rule_3(info)
            new_w_i = self.get_station_weight_1(info[1][0])
            self.routeBuilder.key2Task[key].demand = new_task_demand
            self.routeBuilder.key2Task[key].w_i = new_w_i
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
            return time_label[0], time_label[0] + 3600, sum(bikes_num_info[0][0:7]), 90, cls.get_station_weight_1(
                bikes_num_info[0])
        else:
            return None

    @classmethod
    def get_station_weight_1(cls, bikes_num_info):
        return np.dot(np.array(bikes_num_info), w_i)

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
        return station_loss


if __name__ == '__main__':
    changingRules = ChangingRules()
    changingRules.prepare_route_builder()
    # '''
    changingRules.get_station_info_by_moment(0)
    changingRules.update_existed_tasks()
    changingRules.produce_tasks()
    dynamic_route = [[0, 57, 55, 63, 60, 49, 68, 64, 62, 47, 69, 42, 48, -1, 0],
                     [0, -1, 70, 52, 56, 61, 65, 51, 66, 54, 58, -1, 71, 46, 44, 43, 67, 45, 53, 59, 50, 0]]
    print(changingRules.routeBuilder.get_feasibility(dynamic_route, [0, 1]))
    print(changingRules.routeBuilder.evaluate_solution_by_total_distance(dynamic_route))
    changingRules.routeBuilder.build_initial_solution()
    changingRules.routeBuilder.multiple_neighborhood_search()
    dynamic_route = [[0, 57, 55, 63, 60, 49, 68, 64, 62, 47, 69, 42, 48, -1],
                     [0, -1, 70, 52, 56, 61, 65, 51, 66, 54, 58, -1, 71, 46, 44, 43, 67, 45, 53, 59, 50]]
    print(changingRules.routeBuilder.get_feasibility(dynamic_route, [0, 1]))
    print(changingRules.routeBuilder.evaluate_solution_by_total_distance(dynamic_route))
    print(changingRules.routeBuilder.get_feasibility(changingRules.routeBuilder.best_feas_sol, [0, 1]))
    print(changingRules.routeBuilder.evaluate_solution_by_total_distance(changingRules.routeBuilder.best_feas_sol))
    changingRules.get_routed_result(3600)
    changingRules.stimulate(3600)
    changingRules.get_station_info_by_moment(3600)
    print(changingRules.calculate_loss())
    print(changingRules.calculate_excess_demand())
    print(changingRules.calculate_scheduled_demand())
    print(changingRules.show_bike_distribution())
    '''
    _current_time = 0
    _anticipation_horizon = 600
    _num_time_slices = 6
    for _ts in range(_num_time_slices):
        _current_time = _ts * _anticipation_horizon
        # 获取当前时刻的状态
        changingRules.stimulate(_current_time)

        changingRules.get_station_info_by_moment(_current_time)
        changingRules.update_existed_tasks()
        if _current_time == 0:
            changingRules.produce_tasks()

        changingRules.routeBuilder.build_initial_solution()
        changingRules.routeBuilder.multiple_neighborhood_search()
        # 发送这个时刻得到的，截止到下一个anticipation horizon之前的计划
        # 同时让routeBuilder的状态更新到下一个anticipation horizon开始前的状态
        changingRules.get_routed_result(_current_time + _anticipation_horizon)
    changingRules.stimulate(_num_time_slices * _anticipation_horizon)
    changingRules.get_station_info_by_moment(_num_time_slices * _anticipation_horizon)
    print(changingRules.calculate_loss())
    print(changingRules.routeBuilder.evaluate_solution_by_total_distance(
        [changingRules.routeBuilder.fixedRoutes[0] + [0], changingRules.routeBuilder.fixedRoutes[1] + [0]]))
    print(changingRules.calculate_excess_demand())
    print(changingRules.show_bike_distribution())
    print(changingRules.calculate_scheduled_demand())
    '''
    print('-----------------')
    print('finished')
