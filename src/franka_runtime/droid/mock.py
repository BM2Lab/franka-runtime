"""Kinematic mock FR3 arm and Franka Hand. Not a dynamics model; never connects to hardware."""


class MockArm:
    """Stands in for the motion generator: approaches the (possibly leading) setpoint with
    velocity and acceleration limits, braking so it does not overshoot."""
    hardware_executed = False

    def __init__(self, q0, max_velocity, max_acceleration):
        self.q, self.dq = list(q0), [0.]*7
        self.max_velocity, self.max_acceleration = tuple(max_velocity), tuple(max_acceleration)
        self.stopping = False

    def advance(self, dt_s, target):
        """A stop halts motion immediately (mock only; a real stop decelerates natively)."""
        if self.stopping or target is None or dt_s <= 0:
            self.dq = [0.]*7
            return
        for j in range(7):
            error, v, a = target[j]-self.q[j], self.max_velocity[j], self.max_acceleration[j]
            brake = (2*a*abs(error))**0.5
            desired = (1 if error > 0 else -1)*min(v, brake, abs(error)/dt_s)
            velocity = max(self.dq[j]-a*dt_s, min(self.dq[j]+a*dt_s, desired))
            self.q[j] += velocity*dt_s
            self.dq[j] = velocity

    @property
    def at_rest(self):
        return all(abs(v) < 1e-9 for v in self.dq)


class MockHand:
    hardware_executed = False

    def __init__(self, open_width_m, speed_m_s):
        self.open_width, self.speed = open_width_m, speed_m_s
        self.width = self.target = open_width_m
        self.commands = []

    def command(self, action):
        if action not in ("open", "close"):
            raise ValueError("Gripper command must be open or close")
        self.commands.append(action)
        self.target = self.open_width if action == "open" else 0.

    def advance(self, dt_s):
        step = self.speed*dt_s
        self.width += max(-step, min(step, self.target-self.width))

    @property
    def moving(self):
        return abs(self.width-self.target) > 1e-9
