class Dynamics:
    def init(self, Ixx, Iyy, Izz, Ixy, Iyz, Izx):
        M = [
            [Ixx, Ixy, Izx],
            [Ixy, Iyy, Iyz],
            [Izx, Iyz, Izz],
        ]  # Inertia matrix

        C = []  # Coriolis & centrifugal force vector
        G = [0, 0, -9.8]  # Gravity force vector
