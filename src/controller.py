class PIDController:
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, umax = 0.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.max_output = float(umax)

        self.last_timestamp = 0.0
        self.sp = 0.0
        self.error_sum = 0.0
        self.prev_error = 0.0

        self.u_p = [0]
        self.u_i = [0]
        self.u_d = [0]

    def setGoal(self, goal = 0.0):
        self.sp = float(goal)

    def update(self, mv, timestamp):
        dt = timestamp - self.last_timestamp
        if dt == 0:
            return 0
        
        error = self.sp - mv
        self.last_timestamp = timestamp

        self.error_sum += error * dt

        if self.error_sum > 100:
            self.error_sum = 0
        elif self.error_sum < -100:
            self.error_sum = 0

        delta_error, self.prev_error = error - self.prev_error, error

        p = self.kp * error
        i = self.ki * self.error_sum
        d = self.kd * delta_error / dt

        u = p + i + d

        if u < -self.max_output:
            u = -self.max_output
        elif u > self.max_output:
            u = self.max_output

        self.u_p.append(p)
        self.u_i.append(i)
        self.u_d.append(d)

        return u