import math


def dist(x, y):
    return math.sqrt(x * x + y * y)

def distbetween(x0, y0, x1, y1):
    return dist(x1 - x0, y1 - y0)

def isIntersects(ax, ay, bx, by, cx, cy, dx, dy):
    # see https://stackoverflow.com/a/9997374
    def ccw(ax, ay, bx, by, cx, cy):
        return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)

    return ccw(ax, ay, cx, cy, dx, dy) != ccw(bx, by, cx, cy, dx, dy) and ccw(ax, ay, bx, by, cx, cy) != ccw(ax, ay, bx, by, dx, dy)
