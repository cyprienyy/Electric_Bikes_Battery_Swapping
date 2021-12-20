import numpy as np
import geatpy as ea
import re
import random
import copy
import time
from collections import Counter

file_name_str = r'.\RC202_50'


class Info:

    @staticmethod
    def resolve_self_created_case(filename):
        with open(filename, 'r') as file_to_read:
            pos = []
            while True:
                lines = file_to_read.readline()  # 整行读取数据
                if not lines:
                    break
                    pass
                if re.match(r'\s*[0-9]', lines) is not None:
                    lines = lines.strip()
                    lines = lines.split(',')
                    lines = list(map(float, lines))
                    pos.append(lines)  # 添加新读取的数据
                pass
        pass
        return pos

    def __init__(self):
        self.c_ij = np.zeros((1, 1))
        self.t_ij = np.zeros((1, 1))
        self.satellite = 0
        self.satellite_ser = 0

        self.vehicle_num = 0
        self.vehicle_H = []
        self.vehicle_capacity = []
        self.vehicle_start_locations = []
        self.vehicle_end_locations = []
        self.vehicle_start_load = []

        self.task_num = 0
        self.task_demand = [0]
        self.task_tw_lb = [0]
        self.task_tw_ub = [0]
        self.task_tw_p = [0]
        self.task_ser = [0]
        self.init_with_solomon()

    def init_with_solomon(self):
        pos = self.resolve_self_created_case(file_name_str + '.csv')
        station_num, self.vehicle_num, _capacity, H = map(int, pos[0])
        self.c_ij = np.array(pos[1:station_num + 2])
        self.t_ij = self.c_ij / 1
        self.satellite = 0
        self.satellite_ser = 0

        self.vehicle_H = H * 1
        self.vehicle_capacity = _capacity
        self.vehicle_start_locations = 0
        self.vehicle_end_locations = 0
        self.vehicle_start_load = _capacity

        self.task_num = station_num
        _nodes_info = pos[station_num + 2:]
        _nodes, _t_lower_bound, _t_upper_bound, _demand, _t_ser, _w_i, _H, _loss = zip(*_nodes_info)
        _nodes = list(map(int, _nodes))
        self.task_tw_lb += list(map(int, _t_upper_bound))
        self.task_tw_ub += [H] * station_num
        self.task_demand += list(map(int, _demand))
        self.task_ser += list(map(int, _t_ser))
        self.task_tw_p += list(map(float, _loss))

    def calculate_tw_penalty(self, task, t):
        return self.task_tw_p[task] * (max(t, self.task_tw_lb[task]) - self.task_tw_lb[task]) / (
                self.task_tw_ub[task] - self.task_tw_lb[task])


INFO = Info()

(ALPHA, BETA, RHO, Q) = (1.0, 2.0, 0.5, 100.0)

# 蚁群
ant_num = 100

pheromone_graph = [[10 for _ in range(INFO.task_num + 1)] for _ in range(INFO.task_num + 1)]


# ----------- 蚂蚁 -----------
class Ant(object):

    # 初始化
    def __init__(self, ID):

        self.ID = ID  # ID

        self.load = INFO.vehicle_capacity

        self.path = []
        self.total_distance = 0.0  # 当前路径的总距离

        self.t = 0
        self.current_city = 0  # 当前停留的城市

        self.visited_record = []

        self.__clean_data()  # 随机初始化出生点

    # 初始数据
    def __clean_data(self):

        self.path = []  # 当前蚂蚁的路径
        self.t = 0.0  # 当前路径的总距离
        self.load = INFO.vehicle_capacity
        self.visited_record = [False] * (INFO.task_num + 1)
        self.visited_record[0] = True
        self.total_distance = 0.0
        self.path.append([0])
        self.current_city = 0  # 当前停留的城市

    def __choice_next_city(self):

        _possible_supply = []
        _select_supply_prob = []
        for city, flag in enumerate(self.visited_record):
            if self.t + INFO.c_ij[self.current_city][city] + INFO.c_ij[city][0] <= INFO.vehicle_H \
                    and self.load > INFO.task_demand[city] and flag is False:
                _possible_supply.append(city)
                prob = pow(pheromone_graph[self.current_city][city], ALPHA) * pow(
                    (1.0 / INFO.c_ij[self.current_city][city]), BETA)
                _select_supply_prob.append(prob)

        if (self.load < 0.3 * INFO.vehicle_capacity or len(_possible_supply) == 0) and self.current_city != 0:
            city = 0
            _possible_supply.append(city)
            prob = pow(pheromone_graph[self.current_city][city], ALPHA) * pow(
                (1.0 / INFO.c_ij[self.current_city][city]), BETA)
            _select_supply_prob.append(prob)

        total_prob = sum(_select_supply_prob)

        if total_prob > 0.0:
            # 产生一个随机概率,0.0-total_prob
            temp_prob = random.uniform(0.0, total_prob)
            for i, supply in enumerate(_possible_supply):
                # 轮次相减
                temp_prob -= _select_supply_prob[i]
                if temp_prob < 0.0:
                    next_city = supply
                    return next_city

        return -1

    def __start_another_route(self):
        self.load = INFO.vehicle_start_load
        self.path[-1].append(0)
        self.path.append([0])
        self.t = 0
        self.current_city = 0

    # 移动操作
    def __move_out_of_depot(self, depot):
        self.path[-1].append(depot)
        self.load = INFO.vehicle_capacity

        self.t += INFO.t_ij[self.current_city][depot]
        self.t += INFO.satellite_ser
        self.total_distance += INFO.c_ij[self.current_city][depot]
        self.current_city = depot

    def __move_out_of_demand(self, demand):
        self.path[-1].append(demand)
        self.load -= INFO.task_demand[demand]
        self.visited_record[demand] = True
        self.t += INFO.t_ij[self.current_city][demand]
        self.total_distance += INFO.c_ij[self.current_city][demand]
        self.total_distance += INFO.calculate_tw_penalty(demand, self.t)
        self.t += INFO.task_ser[demand]

        self.current_city = demand

    # 搜索路径
    def search_path(self):
        vehicle_num = 1
        # 初始化数据
        self.__clean_data()
        terminated = True
        # 搜素路径，遍历完所有城市为止
        while all(self.visited_record) is False:
            next_city = self.__choice_next_city()
            if next_city == -1:
                self.__start_another_route()
                vehicle_num += 1
                if vehicle_num > INFO.vehicle_num:
                    break
                continue
            if next_city == 0:
                self.__move_out_of_depot(next_city)
            else:
                self.__move_out_of_demand(next_city)
        self.path[-1].append(0)
        self.t += INFO.t_ij[self.current_city][0]
        self.total_distance += INFO.c_ij[self.current_city][0]
        self.current_city = 0
        if all(self.visited_record) is False:
            self.total_distance += 2000
        return


class VRP(object):

    def __init__(self):
        self.clean_phermone()

        self.ants = [Ant(ID) for ID in range(ant_num)]  # 初始蚁群
        self.best_ant = Ant(-1)  # 初始最优解
        self.best_ant.total_distance = 1 << 31  # 初始最大距离
        self.iter = 1  # 初始化迭代次数

    def clean_phermone(self):
        # 初始城市之间的距离和信息素
        for i in range(INFO.task_num + 1):
            for j in range(INFO.task_num + 1):
                pheromone_graph[i][j] = 10

    # 开始搜索
    def search_path(self, max_iter=100):
        record = []
        non_imp_iter = 0
        while self.iter <= max_iter:
            # 遍历每一只蚂蚁
            non_imp_iter += 1
            for ant in self.ants:
                # 搜索一条路径
                ant.search_path()
                # 与当前最优蚂蚁比较
                if ant.total_distance < self.best_ant.total_distance:
                    # 更新最优解
                    self.best_ant = copy.deepcopy(ant)
                    non_imp_iter = 0
            if non_imp_iter >= 100:
                self.clean_phermone()
            # 更新信息素
            self.__update_pheromone_graph()
            # print(u"迭代次数：", self.iter, u"最佳路径总距离：", self.best_ant.total_distance)
            record.append([self.iter, self.best_ant.total_distance])
            self.iter += 1
        path = self.best_ant.path
        return path, record

    # 更新信息素
    def __update_pheromone_graph(self):

        # 获取每只蚂蚁在其路径上留下的信息素
        temp_pheromone = [[0.0 for _ in range(INFO.task_num + 1)] for _ in range(INFO.task_num + 1)]
        for ant in self.ants:
            for _r in ant.path:
                for start, end in zip(_r, _r[1:]):
                    # 在路径上的每两个相邻城市间留下信息素，与路径总距离反比
                    temp_pheromone[start][end] += Q / ant.total_distance
                    temp_pheromone[end][start] += Q / ant.total_distance

        # 更新所有城市之间的信息素，旧信息素衰减加上新迭代信息素
        for i in range(INFO.task_num + 1):
            for j in range(INFO.task_num + 1):
                pheromone_graph[i][j] = pheromone_graph[i][j] * RHO + temp_pheromone[i][j]


# ----------- 程序的入口处 -----------
def ant_vrp():
    start = time.perf_counter()
    vrp = VRP()
    _path, _record = vrp.search_path(500)
    end = time.perf_counter()
    print('CPU运行时间', end - start)
    print(_path)
    print('最优解', _record[-1][-1])


if __name__ == '__main__':
    # start = time.perf_counter()
    # vrp = VRP()
    # _path, _record = vrp.search_path(1000)
    # _supply = []
    # _demand = []
    # _gene = []
    # for _rt in _path:
    #     for _i in _rt:
    #         if _i in supplys and _i not in _supply:
    #             _supply.append(_i)
    #         if _i in demands and _i not in _demand:
    #             _demand.append(_i)
    # for _i in _supply:
    #     _gene.append(supplys.index(_i))
    # for _i in _demand:
    #     _gene.append(demands.index(_i))
    # print(_gene)
    # end = time.perf_counter()
    # print('CPU运行时间', end - start)
    # with open('ant.csv', 'w', newline='') as f:
    #     f_csv = csv.writer(f)
    #     f_csv.writerows(_record)
    for i in range(5):
        print('迭代次数', i)
        ant_vrp()
    # ant_vrp()
