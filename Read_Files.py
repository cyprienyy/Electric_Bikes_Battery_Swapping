import re
import numpy as np
from scipy.spatial.distance import cdist
from Routes import RouteBuilder


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


if __name__ == '__main__':
    file_path = r'.\solomon_25\C101.txt'
    _info, _mat = read_single_soloman(file_path)
    _vehicle_num, _capacity, _dis_mat, _demand, _t_win, _t_ser = resolve_soloman(_info, _mat)
    _dis_mat = np.around(_dis_mat, 1)
    _vehicle_num = 3
    routeBuilder = RouteBuilder(_dis_mat, _dis_mat)
    routeBuilder.add_empty_route([0]*_vehicle_num, [0]*_vehicle_num, [0]*_vehicle_num, [_t_win[0, 1]]*_vehicle_num)
    routeBuilder.add_tasks(list(range(1, 26)), _t_win[1:, 0], _t_win[1:, 1], _demand[1:], _t_ser[1:])
    routeBuilder.build_initial_solution()
    routeBuilder.print_sol()
    print(routeBuilder.best_feas_obj)
    print(routeBuilder.best_feas_sol)
    print('------------------')
    routeBuilder.multiple_neighborhood_search()
    routeBuilder.print_sol()
    print(routeBuilder.evaluate_solution(routeBuilder.best_feas_sol))
    print(routeBuilder.best_feas_sol)