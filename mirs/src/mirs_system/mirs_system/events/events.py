class Event:
    EVENTS_RESUME = 0
    EVENT_START = 1
    EVENT_PAUSE = 2
    EVENT_START_RECORD = 3
    EVENT_END_RECORD = 4
    EVENT_EXECUTE_TASK = 5
    EVENT_EXIT = 6

    CUR_EVENT = None

    MAIN_QUEUE = []

    @staticmethod
    def fire(event):
        Event.CUR_EVENT = event
        Event.notify()

    @staticmethod
    def notify():
        for process in Event.MAIN_QUEUE:
            process.set_status(Event.CUR_EVENT)


class Process:
    PROCESSES = []

    def __init__(self, id, status=None):
        self.id = id
        self.status = status
        Process.PROCESSES.append(self)

    @staticmethod
    def set_status(id, status):
        procees = Process.get_process(id)
        procees.status = status

    @staticmethod
    def get_status(id):
        process = Process.get_process(id)
        return process.status

    @staticmethod
    def get_process(id: int):
        for process in Process.PROCESSES:
            if process.id == id:
                return process
