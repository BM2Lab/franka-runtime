"""Franka Hand command intent: commands only on closure transitions across the contract threshold."""


class GripperIntent:
    def __init__(self, closure_map, measured_width_m):
        self.map = closure_map
        self.state = self.map.command_from_closure(self.map.closure_from_width(measured_width_m))

    def update(self, closure):
        """Return "close"/"open" when the commanded state changes, else None."""
        desired = self.map.command_from_closure(closure)
        if desired == self.state:
            return None
        self.state = desired
        return desired
