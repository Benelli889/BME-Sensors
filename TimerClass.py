from datetime import datetime
from datetime import timedelta

class TimerClass:
    STOPPED = 0
    STARTED = 1
    TimerStarted = 0
    dtTimerDuration = 0

    def __INIT__(self):
        self.TimerStarted = 0
        self.tFutureTime = 0

    def State(self):
        return self.TimerStarted

    def TimerRunUp(self):
        if datetime.now() >= self.tFutureTime:
            self.TimerStarted = 0
            return True

    def Start(self):
        self.tFutureTime = datetime.now() + self.dtTimerDuration
        self.TimerStarted = 1

    def SetTimerDuration(self, TimerDuration):
        self.dtTimerDuration = TimerDuration
        