import numpy as np

c_ij = np.random.randint(0, 5, size=(9, 9))


def get_feasibility_of_balance_route(q_i, route, capacity, tw_required):
    """
    :param q_i: list[int], q_i>0 is removal, q_i<0 is delivery.
    :param route: list[int].
    :param capacity: int, Q.
    :param tw_required: bool, controls content returned.
    :return: route is load feasible? and (optional) load window at depot.
    """
    lambda_p = 0
    lambda_min = 0
    lambda_max = 0
    for i, loc in enumerate(route):
        lambda_p += q_i[i]
        lambda_min = min(lambda_p, lambda_min)
        lambda_max = max(lambda_p, lambda_max)
        if (q_i[i] + capacity - lambda_max) < (q_i[i] - lambda_min):
            if tw_required:
                return False, (None, None)
            else:
                return False
    if tw_required:
        return True, (-lambda_min, capacity - lambda_max)
    else:
        return True


def get_skip_for_feasible_route(a_i, b_i, q_i, route, capacity):
    # {tuple(skipped_points) : (theta_i_0, theta_i_1)}.
    labels_list = [{} for _ in range(len(route))]
    labels_list[0][(-1,)] = (0, get_feasibility_of_balance_route(q_i, route, capacity, True)[1][1])
    for i, loc in enumerate(route):
        for sk_p_tuple, load_info_tuple in labels_list[i].items():
            theta_i_0, theta_i_1 = load_info_tuple
            sk_p_list = list(sk_p_tuple)
            for j in range(i + 1, len(route)):
                theta_sum_tmp = theta_i_0 + theta_i_1 + q_i[j]
                theta_1_tmp = theta_i_1 + min(q_i[j] + theta_i_0, a_i[j])
                theta_0_tmp = theta_sum_tmp - theta_1_tmp
                add_label = True
                for key in labels_list[j].keys():
                    if set(sk_p_list).issubset(set(key)):
                        if j == len(route) - 1:
                            add_label = False
                        elif theta_1_tmp <= labels_list[j][key][1]:
                            add_label = False
                if add_label:
                    labels_list[j][tuple(sk_p_list)] = (theta_0_tmp, theta_1_tmp)
                if j == len(route) - 1 or theta_i_1 < b_i[j] - q_i[j]:
                    break
                else:
                    theta_i_1 = theta_i_1 - (b_i[j] - q_i[j])
                    theta_i_0 = theta_sum_tmp - theta_i_1
                    sk_p_list.append(route[j])
    return list(labels_list[-1].keys())


def get_most_saved_distance(balance_routes, battery_change_route, q_i, a_i, b_i, capacity):
    skip_plans_by_route = []
    for b_r in balance_routes:
        temp_q_i = []
        temp_a_i = []
        temp_b_i = []
        for loc in b_r:
            temp_q_i.append(q_i[loc])
            temp_a_i.append(a_i[loc])
            temp_b_i.append(b_i[loc])
        skip_plans_by_route.append(get_skip_for_feasible_route(temp_a_i, temp_b_i, temp_q_i, b_r, capacity))
    print(skip_plans_by_route)
    return backtracking_best_combination(skip_plans_by_route, battery_change_route)


def get_greedy_saved_distance(balance_routes, battery_change_route, q_i, a_i, b_i, capacity):
    distance_saved = np.zeros(len(battery_change_route), np.int32)
    for i, loc in enumerate(battery_change_route):
        if loc != 0:
            distance_saved[i] = c_ij[battery_change_route[i - 1], loc] + c_ij[loc, battery_change_route[i + 1]]
        else:
            distance_saved[i] = 0

    balance_routes_feasibility = []
    balance_routes_load = []
    for b_r in balance_routes:
        temp_q_i = []
        temp_a_i = []
        temp_b_i = []
        for loc in b_r:
            temp_q_i.append(q_i[loc])
            temp_a_i.append(a_i[loc])
            temp_b_i.append(b_i[loc])
        b_r_feas, b_r_load = get_feasibility_of_balance_route(temp_q_i, b_r, capacity, True)
        if b_r_feas:
            balance_routes_feasibility.append(True)
            theta_0_tmp = 0
            theta_1_tmp = b_r_load[1]
            balance_routes_load.append([(theta_0_tmp, theta_1_tmp)])
            for j in range(1, len(b_r)):
                theta_sum_tmp = theta_0_tmp + theta_1_tmp + temp_q_i[j]
                theta_1_tmp = theta_1_tmp + min(temp_q_i[j] + theta_0_tmp, temp_a_i[j])
                theta_0_tmp = theta_sum_tmp - theta_1_tmp
                balance_routes_load[-1].append((theta_0_tmp, theta_1_tmp))
        else:
            balance_routes_feasibility.append(False)
            balance_routes_load.append([])

    skipped_points = set()
    while np.max(distance_saved) > 0:
        index = np.argmax(distance_saved)
        loc = battery_change_route[index]
        found = False
        skip_possible = True
        skip_points_location = (0, 0)
        new_b_r_load = []
        for i, b_r in enumerate(balance_routes):
            for j, pos in enumerate(b_r):
                if pos == loc:
                    skip_points_location = (i, j)
                    if balance_routes_feasibility[i] is True:
                        theta_0_tmp, theta_1_tmp = balance_routes_load[i][j - 1]
                        for k in range(j, len(b_r)):
                            if k == j or b_r[k] in skipped_points:
                                if theta_1_tmp >= b_i[b_r[k]] - q_i[b_r[k]]:
                                    theta_sum_tmp = theta_0_tmp + theta_1_tmp + q_i[b_r[k]]
                                    theta_1_tmp = theta_1_tmp - (b_i[b_r[k]] - q_i[b_r[k]])
                                    theta_0_tmp = theta_sum_tmp - theta_1_tmp
                                    new_b_r_load.append((theta_0_tmp, theta_1_tmp))
                                else:
                                    skip_possible = False
                                    break
                            else:
                                theta_sum_tmp = theta_0_tmp + theta_1_tmp + q_i[b_r[k]]
                                theta_1_tmp = theta_1_tmp + min(q_i[b_r[k]] + theta_0_tmp, a_i[b_r[k]])
                                theta_0_tmp = theta_sum_tmp - theta_1_tmp
                                new_b_r_load.append((theta_0_tmp, theta_1_tmp))
                    else:
                        skip_possible = False
                    found = True
                    break
            if found:
                break
        if skip_possible is True:
            skipped_points.add(loc)
            balance_routes_load[skip_points_location[0]][skip_points_location[1]:] = new_b_r_load.copy()

        distance_saved[index] = 0
        low_index = index - 1
        high_index = index + 1
        while low_index > 0 and distance_saved[low_index] == 0:
            low_index = low_index - 1
        while high_index < len(battery_change_route) - 1 and distance_saved[high_index] == 0:
            high_index = high_index + 1
        if low_index > 0:
            distance_saved[low_index] = distance_saved[low_index] + c_ij[
                battery_change_route[low_index], battery_change_route[high_index]] - c_ij[
                                            battery_change_route[low_index], battery_change_route[index]]
        if high_index < len(battery_change_route) - 1:
            distance_saved[high_index] = distance_saved[high_index] + c_ij[
                battery_change_route[low_index], battery_change_route[high_index]] - c_ij[
                                             battery_change_route[index], battery_change_route[high_index]]

    return skipped_points


def backtracking_best_combination(skip_plans_by_route, battery_change_route):
    def generate(combination, i, shortest_distance):
        if i == len(skip_plans_by_route):
            new_s_dis = evaluate_saved_distance(combination, battery_change_route)
            print(combination)
            if new_s_dis < shortest_distance[0]:
                shortest_distance[0] = new_s_dis
        else:
            for plan_j in skip_plans_by_route[i]:
                combination = combination + list(plan_j)
                generate(combination, i + 1, shortest_distance)
                combination = combination[:len(combination) - len(plan_j)]

    ans = [float('inf')]
    generate([], 0, ans)
    return ans[0]


def evaluate_saved_distance(skipped_points, battery_change_route):
    dis = 0
    skipped_points_set = set(skipped_points)
    i = battery_change_route[0]
    for loc in battery_change_route[1:]:
        if loc not in skipped_points_set:
            dis = dis + c_ij[i, loc]
            i = loc
    return dis


if __name__ == '__main__':
    print('Calculate_saved_distance Main')
    _q_i = [0, -4, 5, 2, 1, 2, 4, -3, -5]
    # _q_i_0 = [0, -4, 5, 2, 1, 0]
    # _q_i_1 = [0, 2, 4, -3, -5, 0]
    _capacity = 10
    _balance_route = [[0, 1, 2, 3, 4, 0], [0, 5, 6, 7, 8, 0]]
    _a_i = [0] + [5, 1, 5, 2] + [2, 3, 1, 0]
    _b_i = [0] + [9, 5, 2, 1] + [3, 3, 7, 0]
    _tsp_route = [0, 1, 2, 3, 4, 5, 6, 7, 8, 0]
    print(get_most_saved_distance(_balance_route, _tsp_route, _q_i, _a_i, _b_i, _capacity))
    print(get_greedy_saved_distance(_balance_route, _tsp_route, _q_i, _a_i, _b_i, _capacity))
    print('Main')
