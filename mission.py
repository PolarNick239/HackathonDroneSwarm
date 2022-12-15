class Mission:

    def __init__(self, key, total_time, x, y):
        self.key = key
        self.total_time = total_time
        self.time_to_finish_left = total_time
        self.x, self.y = x, y
