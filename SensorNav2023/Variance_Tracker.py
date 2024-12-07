class Variance_Tracker:
    def __init__(self):
        self.n = 0
        self.mean = 0
        self.M2 = 0

    def update_var(self, acc):
        self.n += 1
        delta1 = acc - self.mean
        self.mean += delta1 / self.n
        delta2 = acc - self.mean
        self.M2 += delta1 * delta2

        if self.n < 2:
            return 0

        return (self.M2 / (self.n - 1))

