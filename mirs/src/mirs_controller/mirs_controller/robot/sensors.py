from system.hardware.connection import Connection


class PositionSensor:
    def __init__(self, name):
        self.name = name

    def sensor_enable(self, sampling_period):  # milliseconds
        pass

    def sensor_disable(self):
        pass

    def sensor_get_sampling_period(self):
        pass

    def sensor_get_value(self):  # rad or meters
        pass

    def sensor_get_type(self):
        pass

    def sensor_get_motor(self):
        pass

    def sensor_get_brake(self):
        pass


class DistanceSensor:
    def __init__(self):
        self.DistanceSensorType = {
            "GENERIC": 0,
            "INFRA_RED": 1,
            "SONAR": 2,
            "LASER": 3
        }

    def enable(self, sampling_period):
        pass

    def disable(self):
        pass

    def get_sampling_period(self):
        pass

    def get_value(self):
        pass

    def get_max_value(self):
        pass

    def get_min_value(self):
        pass

    def get_aperture(self):
        pass

    def get_lookup_table_size(self):
        pass

    def get_lookup_table(self):
        pass

    def get_type(self):
        pass
