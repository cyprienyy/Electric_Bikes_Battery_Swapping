import numpy as np
from Read_Files import resolve_self_created_case
from Routes import RouteBuilder

"""
initialize infFactor
initialize ban list
initialize start neighborhood
currentIncumbent := solution from tour construction
for maxIter do
    for each neighborhood do
        for maxNbIter do
            N := neighbor of currentIncumbent
            currentIncumbent := min(n){cost(n) | ∀n ∈ N : cost(n) ∈/ ban list}
            update infFactor
            update ban list
            if reached maxNbNonImpIter then
                change neighborhood
                reset currentIncumbent to best feasible solution found so far
                reset infFactor
                break
            end if
        end for
        change neighborhood
        reset currentIncumbent to best feasible solution found so far
        reset infFactor
    end for
    if reached maxNonImpIter then
        break
    end if
end for
"""
# 906.6625090161937
from Read_Files import resolve_station_inventory

routes = [[0, 38, 40, 42, 44, -1, 48, 47, 46, 49, 36, 31, 29, -1, 35, 32, 34, 30, 37, 33, -1, 50, 51, 28, 27, -1, 0], [0, 39, 43, 41, -1, 45, 0]]

filepath = r'.\C101.csv'
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
print(routeBuilder.get_feasibility(routes, range(_vehicle_num)))
print(routeBuilder.evaluate_solution(routes, range(_vehicle_num)))
