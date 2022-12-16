import math


def dist(x, y):
    return math.sqrt(x * x + y * y)

def distbetween(x0, y0, x1, y1):
    return dist(x1 - x0, y1 - y0)
