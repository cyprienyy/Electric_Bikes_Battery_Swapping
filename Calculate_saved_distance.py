def get_feasibility_of_balance_route(q_i, route, Q):
    lambda_p = 0
    lambda_min = 0
    lambda_max = 0
    for i, loc in enumerate(route):
        lambda_p += q_i[i]
        lambda_min = min(lambda_p, lambda_min)
        lambda_max = max(lambda_p, lambda_max)
        if (q_i[i] + Q - lambda_max) < (q_i[i] - lambda_min):
            return False
    return True


def get_skip_for_feasible_route(a_i, b_i, q_i, route, Q):
    return []
