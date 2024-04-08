from PID import PID


class Packet:
    def __init__(self):
        self.pid = PID()
        self.plot_data = [[] * 6]
        self.resetEncoders = False

    def unpack(self):
        return [
            self.pid,
            self.resetEncoders,
        ]
