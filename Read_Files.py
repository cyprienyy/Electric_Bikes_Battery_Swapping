import re
import numpy as np
from scipy.spatial.distance import cdist
from Routes import RouteBuilder, RouteBuilderForSoloman, RouteBuilderByDistance
from collections import Counter
import csv
import time


def read_single_soloman(filename):
    with open(filename, 'r') as file_to_read:
        pos = []
        while True:
            lines = file_to_read.readline()  # 整行读取数据
            if not lines:
                break
                pass
            if re.match(r'\s*[0-9]', lines) is not None:
                lines = lines.strip()
                lines = lines.split()
                lines = list(map(int, lines))
                pos.append(lines)  # 添加新读取的数据
            pass
    pass
    info = pos[0]
    mat = pos[1:]
    return info, mat


def get_dis_mat(cor):
    return cdist(cor, cor)


def resolve_soloman(info, mat):
    vehicle_num, capacity = info
    mat = np.array(mat)
    dis_mat = get_dis_mat(mat[:, 1:3])
    demand = mat[:, 3]
    t_win = mat[:, 4:6]
    t_ser = mat[:, 6]
    return vehicle_num, capacity, dis_mat, demand, t_win, t_ser


def solve_soloman():
    file_path = r'.\solomon_50\R112.txt'
    _info, _mat = read_single_soloman(file_path)
    _vehicle_num, _capacity, _dis_mat, _demand, _t_win, _t_ser = resolve_soloman(_info, _mat)
    _dis_mat = np.around(_dis_mat, 1)
    _vehicle_num = 9
    routeBuilder = RouteBuilderForSoloman(_dis_mat, _dis_mat)
    routeBuilder.add_empty_route([0] * _vehicle_num, [0] * _vehicle_num, [0] * _vehicle_num,
                                 [_t_win[0, 1]] * _vehicle_num, [_capacity] * _vehicle_num)
    routeBuilder.add_tasks(list(range(1, 51)), _t_win[1:, 0], _t_win[1:, 1], _demand[1:], _t_ser[1:], [1] * 50)
    routeBuilder.build_initial_solution()
    print(routeBuilder.get_feasibility(routeBuilder.routes, range(_vehicle_num)))
    routeBuilder.print_sol()
    print(routeBuilder.get_sol_schedule())
    print(routeBuilder.best_feas_obj)
    routeBuilder.multiple_neighborhood_search()
    routeBuilder.print_sol()
    print(routeBuilder.get_sol_schedule())
    print(routeBuilder.best_feas_obj)
    return


def resolve_station_inventory(station_inventory_path='.\station_inventory.npy'):
    station_inventory = np.load(station_inventory_path)

    for bike_num in station_inventory:
        bike_counter = Counter()
        bl = 0
        for i in bike_num:
            bike_counter[bl] = i
            bl += 10
        yield bike_counter


def resolve_instancesBRP(filepath='1Bari30.txt'):
    with open(filepath, 'r') as file_to_read:
        pos = []
        while True:
            lines = file_to_read.readline()  # 整行读取数据
            if not lines:
                break
                pass
            if re.match(r'\s*[0-9]', lines) is not None:
                lines = lines.strip()
                lines = lines.split()
                lines = list(map(resolve_str_2_int, lines))
                pos.append(lines)  # 添加新读取的数据
            pass
    pass
    return pos[0][0], np.array(pos[1]), np.array(pos[2]), np.array(pos[3]), pos[4][0], np.array(pos[5:])


def resolve_str_2_int(x):
    if x == '1e+009' or x == '1E+009':
        return 1000000
    else:
        return int(x)


def transform_soloman(filepath=r'.\solomon_50\RC202.txt'):
    _info, _mat = read_single_soloman(filepath)
    _vehicle_num, _capacity, _dis_mat, _demand, _t_win, _t_ser = resolve_soloman(_info, _mat)

    _station_num = _dis_mat.shape[0] - 1

    _vehicle_num = 2

    # _capacity = _capacity
    _capacity = 200

    H = _t_win[0, 1] * 2

    _dis_mat = np.around(_dis_mat, 1) * 1

    _nodes = list(range(1, _station_num + 1))
    _t_lower_bound = [0] * _station_num
    _t_upper_bound = _t_win[1:, 1].tolist()
    _t_upper_bound = [i * 1.5 for i in _t_upper_bound]
    _demand = _demand[1:].tolist()
    _t_ser = _t_ser[1:].tolist()
    _w_i = [1] * _station_num
    _H = [H] * _station_num

    _min_dis = np.min(_dis_mat[_dis_mat > 0])
    _loss = _min_dis * (1 + np.random.rand(_station_num)) * 2
    _loss = np.around(_loss, 1)
    _loss.tolist()

    _nodes_info = list(zip(_nodes, _t_lower_bound, _t_upper_bound, _demand, _t_ser, _w_i, _H, _loss))

    with open('RC202_50.csv', 'w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([_station_num, _vehicle_num, _capacity, H])
        writer.writerows(_dis_mat.tolist())
        writer.writerows(_nodes_info)

    return


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


def solve_self_created_case(filepath=r'.\C101.csv'):
    pos = resolve_self_created_case(filepath)
    _station_num, _vehicle_num, _capacity, H = map(int, pos[0])
    _c_ij = np.array(pos[1:_station_num + 2])
    _t_ij = _c_ij / 1
    _nodes_info = pos[_station_num + 2:]
    _nodes, _t_lower_bound, _t_upper_bound, _demand, _t_ser, _w_i, _H, _loss = zip(*_nodes_info)
    _nodes = list(map(int, _nodes))
    _t_lower_bound = list(map(int, _t_lower_bound))
    _t_upper_bound = list(map(int, _t_upper_bound))
    _demand = list(map(int, _demand))
    _t_ser = list(map(int, _t_ser))
    _w_i = list(map(int, _w_i))
    _H = list(map(int, _H))
    routeBuilder = RouteBuilder(_c_ij, _t_ij)
    routeBuilder.add_empty_route([0] * _vehicle_num, [0] * _vehicle_num, [0] * _vehicle_num,
                                 [H] * _vehicle_num, [_capacity] * _vehicle_num)
    routeBuilder.add_tasks(_nodes, _t_lower_bound, _t_upper_bound, _demand, _t_ser, _w_i, _H, _loss)

    time_start = time.time()
    routeBuilder.build_initial_solution()
    print(routeBuilder.get_feasibility(routeBuilder.routes, range(_vehicle_num)))
    routeBuilder.print_sol()
    # print(routeBuilder.get_sol_schedule())
    print(routeBuilder.best_feas_obj)
    routeBuilder.multiple_neighborhood_search()
    print(routeBuilder.best_feas_obj)
    end_time = time.time()
    print('用时', end_time - time_start)
    routeBuilder.print_sol()
    print(routeBuilder.get_feasibility(routeBuilder.routes, range(_vehicle_num)))
    print(routeBuilder.get_sol_schedule())
    print(routeBuilder.evaluate_solution_by_total_distance(routeBuilder.routes))
    print(routeBuilder.evaluate_solution_by_time_func_of_task(routeBuilder.routes, range(_vehicle_num)))

    # routes = [[0, 32, 39, 28, 31, 44, 33, 34, 46, 35, 29, 50, 38, 51, 27, 0],
    #           [0, 47, 48, 30, 49, 40, 42, 43, 45, 37, 36, 41, 0]]
    # print(routeBuilder.get_feasibility(routes, range(_vehicle_num)))
    # print(routeBuilder.evaluate_solution_by_total_distance(routes))
    # print(routeBuilder.evaluate_solution(routes, range(_vehicle_num)))
    return


if __name__ == '__main__':
    transform_soloman()
    # solve_self_created_case()
    # solve_soloman()
    '''
    print(routeBuilder.best_feas_sol)
    print('------------------')
    routeBuilder.multiple_neighborhood_search()
    routeBuilder.print_sol()
    print(routeBuilder.evaluate_solution(routeBuilder.best_feas_sol))
    print(routeBuilder.best_feas_sol)
    print('++++++++++++++++++')
    '''
    print('finished')
