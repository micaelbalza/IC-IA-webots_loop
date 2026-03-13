import math


def euclidean_distance(p1, p2):
    return math.dist(p1, p2)


def c_space(pdp, robot_radius, alpha):
    if alpha < 1:
        alpha = 1

    num_points = len(pdp)
    if num_points < 2:
        return list(pdp)

    new_polygon = []

    first_direction = _normalize(_subtract(pdp[1], pdp[0]))
    first_normal = [-first_direction[1], first_direction[0]]
    new_polygon.append(_add(pdp[0], _scale(first_normal, alpha * robot_radius)))

    for index in range(num_points - 1):
        direction = _normalize(_subtract(pdp[index + 1], pdp[index]))
        normal = [-direction[1], direction[0]]

        p1_new = _add(pdp[index], _scale(normal, alpha * robot_radius))
        p2_new = _add(pdp[index + 1], _scale(normal, alpha * robot_radius))

        if index < num_points - 2:
            next_direction = _normalize(_subtract(pdp[index + 2], pdp[index + 1]))
            next_normal = [-next_direction[1], next_direction[0]]
            next_p1_new = _add(pdp[index + 1], _scale(next_normal, alpha * robot_radius))
            next_p2_new = _add(pdp[index + 2], _scale(next_normal, alpha * robot_radius))

            intersection = _line_intersection(p1_new, p2_new, next_p1_new, next_p2_new)
            if intersection is not None:
                new_polygon.append(intersection)

    last_direction = _normalize(_subtract(pdp[-1], pdp[-2]))
    last_normal = [-last_direction[1], last_direction[0]]
    new_polygon.append(_add(pdp[-1], _scale(last_normal, alpha * robot_radius)))
    return new_polygon


def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    count = len(polygon)
    if count < 3:
        return False

    j = count - 1
    for i in range(count):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) if (yj - yi) != 0 else 1e-12) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def segments_intersect(p1, p2, p3, p4):
    return (
        _orientation(p1, p3, p4) != _orientation(p2, p3, p4)
        and _orientation(p1, p2, p3) != _orientation(p1, p2, p4)
    )


def _orientation(p1, p2, p3):
    value = (p2[1] - p1[1]) * (p3[0] - p2[0]) - (p2[0] - p1[0]) * (p3[1] - p2[1])
    if value == 0:
        return 0
    return 1 if value > 0 else -1


def _line_intersection(p1, p2, p3, p4):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4

    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denominator) < 1e-12:
        return None

    det1 = x1 * y2 - y1 * x2
    det2 = x3 * y4 - y3 * x4
    px = (det1 * (x3 - x4) - (x1 - x2) * det2) / denominator
    py = (det1 * (y3 - y4) - (y1 - y2) * det2) / denominator
    return [px, py]


def _subtract(p1, p2):
    return [p1[0] - p2[0], p1[1] - p2[1]]


def _add(p1, p2):
    return [p1[0] + p2[0], p1[1] + p2[1]]


def _scale(vector, scalar):
    return [vector[0] * scalar, vector[1] * scalar]


def _normalize(vector):
    norm = math.hypot(vector[0], vector[1])
    if norm == 0:
        return [0.0, 0.0]
    return [vector[0] / norm, vector[1] / norm]
