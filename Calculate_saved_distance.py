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
                labels_list[i][tuple(sk_p_list)] = (theta_0_tmp, theta_1_tmp)
                if j == len(route) - 1 or theta_i_1 < b_i[j] - q_i[j]:
                    break
                sk_p_list.append(j)
                # To do: add the calculation of new theta_i_1, theta_i_0
    return


if __name__ == '__main__':
    _q_i = [0, -3, 2, 4, -7, 0]
    _capacity = 10
    _route = [0, 1, 2, 3, 4, 0]
    print(get_feasibility_of_balance_route(_q_i, _route, _capacity, True))
    _a_i = [0] * 6
    _b_i = [0] * 6
    get_skip_for_feasible_route(_a_i, _b_i, _q_i, _route, _capacity)
    print('Main')
