import math

from .geometry import point_in_polygon, segments_intersect


def beta(po_size):
    return 1 if po_size > 0 else 0


def d_of(candidate, objective):
    return math.dist(candidate, objective)


def d_o(candidate, obstacles):
    if not obstacles:
        return 1_000_000_000.0
    return min(math.dist(candidate, obstacle) for obstacle in obstacles)


def c_j(candidate, pr, radius=1.0):
    if len(pr) <= 1:
        return 1.0

    visited_previously = any(math.dist(candidate, point) <= radius for point in pr[:-1])
    return 1000.0 if visited_previously else 1.0


def a_j(candidate, pr, pdp_space):
    current_position = pr[-1]
    if not point_in_polygon(candidate, pdp_space):
        return float("inf")

    for index in range(len(pdp_space)):
        next_index = (index + 1) % len(pdp_space)
        if segments_intersect(candidate, current_position, pdp_space[index], pdp_space[next_index]):
            return float("inf")
    return 0.0


def g_j(candidate, pdp_space, po, po_size, pr, objective):
    goal_term = d_of(candidate, objective)
    angular_term = a_j(candidate, pr, pdp_space)

    if beta(po_size) == 1:
        obstacle_distance = d_o(candidate, po)
        obstacle_term = 1.0 / obstacle_distance if obstacle_distance != 0 else float("inf")
        revisit_term = c_j(candidate, pr)
    else:
        obstacle_term = 0.0
        revisit_term = 0.0

    return goal_term + obstacle_term + revisit_term + angular_term
