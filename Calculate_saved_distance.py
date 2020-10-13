import numpy as np


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


def backtracking_best_combination(skip_plans_by_route, battery_change_route):
    def generate(combination, i, shortest_distance):
        if i == len(skip_plans_by_route):
            new_s_dis = evaluate_saved_distance(combination, battery_change_route)
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


def evaluate_saved_distance(skipped_points, battery_change_route, c_ij=np.random.randint(0, 5, size=(9, 9))):
    dis = 0
    skipped_points_set = set(skipped_points)
    i = battery_change_route[0]
    for loc in battery_change_route[1:]:
        if loc not in skipped_points_set:
            dis = dis + c_ij[i, loc]
            i = loc
    return dis


if __name__ == '__main__':
    _q_i = [0, -4, 5, 2, 1, 2, 4, -3, -5]
    # _q_i_0 = [0, -4, 5, 2, 1, 0]
    # _q_i_1 = [0, 2, 4, -3, -5, 0]
    _capacity = 10
    _balance_route = [[0, 1, 2, 3, 4, 0], [0, 5, 6, 7, 8, 0]]
    _a_i = [0] + [5, 1, 5, 2] + [2, 3, 1, 0]
    _b_i = [0] + [9, 5, 2, 1] + [3, 3, 7, 0]
    _tsp_route = [0, 1, 2, 3, 4, 5, 6, 7, 8, 0]
    _skip_plans_by_route = [[(-1, 1, 2), (-1, 2, 3)], [(-1, 6, 5), (-1, 7, 9)]]
    backtracking_best_combination(_skip_plans_by_route, _tsp_route)
    print('Main')
