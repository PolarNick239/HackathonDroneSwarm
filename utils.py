import math
import numpy as np


def dist(x, y):
    return math.sqrt(x * x + y * y)

def distbetween(x0, y0, x1, y1):
    return dist(x1 - x0, y1 - y0)

def isIntersects(ax, ay, bx, by, cx, cy, dx, dy):
    # see https://stackoverflow.com/a/9997374
    def ccw(ax, ay, bx, by, cx, cy):
        return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)

    return ccw(ax, ay, cx, cy, dx, dy) != ccw(bx, by, cx, cy, dx, dy) and ccw(ax, ay, bx, by, cx, cy) != ccw(ax, ay, bx, by, dx, dy)

def distancePointToSegment(px, py, ax, ay, bx, by):
    p = np.float32([px, py])
    a = np.float32([ax, ay])
    b = np.float32([bx, by])

    point = p
    line = [a, b]
    # see https://stackoverflow.com/a/45483585
    unit_line = line[1] - line[0]
    norm_unit_line = unit_line / np.linalg.norm(unit_line)

    # compute the perpendicular distance to the theoretical infinite line
    segment_dist = (
            np.linalg.norm(np.cross(line[1] - line[0], line[0] - point)) /
            np.linalg.norm(unit_line)
    )

    diff = (
            (norm_unit_line[0] * (point[0] - line[0][0])) +
            (norm_unit_line[1] * (point[1] - line[0][1]))
    )

    x_seg = (norm_unit_line[0] * diff) + line[0][0]
    y_seg = (norm_unit_line[1] * diff) + line[0][1]

    endpoint_dist = min(
        np.linalg.norm(line[0] - point),
        np.linalg.norm(line[1] - point)
    )

    # decide if the intersection point falls on the line segment
    lp1_x = line[0][0]  # line point 1 x
    lp1_y = line[0][1]  # line point 1 y
    lp2_x = line[1][0]  # line point 2 x
    lp2_y = line[1][1]  # line point 2 y
    is_betw_x = lp1_x <= x_seg <= lp2_x or lp2_x <= x_seg <= lp1_x
    is_betw_y = lp1_y <= y_seg <= lp2_y or lp2_y <= y_seg <= lp1_y
    if is_betw_x and is_betw_y:
        return segment_dist
    else:
        # if not, then return the minimum distance to the segment endpoints
        return endpoint_dist

def simplifyPath(xys, max_error):
    progress = True
    while progress:
        progress = False
        best_i = -1
        best_error = max_error
        for i in range(1, len(xys) - 1):
            ax, ay = xys[i - 1]
            px, py = xys[i]
            bx, by = xys[i + 1]
            error = distancePointToSegment(px, py, ax, ay, bx, by)
            if error < best_error:
                best_i = i
                best_error = error
                if error < max_error / 100.0:
                    break
        if best_i != -1:
            xys = xys[:best_i] + xys[best_i + 1:]
            progress = True
    return xys
