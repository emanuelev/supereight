from math import sqrt


def golden_section_search(lower, upper, epsilon, func):
    """ Do a golden section search for minimum between lower and upper to epsilon precision using func to do evaluation
    :param lower:
    :param upper:
    :param epsilon:
    :param func:
    :return:
    """
    phi = (-1.0 + sqrt(5)) / 2.0

    f_lower = func(lower)
    f_upper = func(upper)

    while abs(upper - lower) > epsilon:
        a = upper - phi * (upper - lower)
        b = lower + phi * (upper - lower)

        f_a = func(a)
        f_b = func(b)

        if f_a < f_b:
            upper, f_upper = b, f_b
        else:
            lower, f_lower = a, f_a

    center = (upper + lower) / 2
    return center, func(center)
