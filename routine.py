from threading import Thread as ParralelClass
from threading import Event, Condition



class Routine(ParralelClass):

    def __init__(self, object_with_io, callback, condition, freq):
        ParralelClass.__init__(self)
        self.period = 1. / freq
        self.object_with_io = object_with_io
        self.callback = callback
        self.condition = condition
        self._running = Event()
        self._running.clear()
        self._to_terminate = Event()
        self._to_terminate.clear()

    def run(self):
        while True:
            if self._to_terminate.is_set():
                break
            start_time = self.object_with_io.io.get_simulation_current_time()
            if self._running.is_set():
                self.condition.acquire()
                self.loop_core()
                self.condition.release()
            time_to_wait = self.period + start_time - self.object_with_io.io.get_simulation_current_time()
            if time_to_wait >= 0.:
                self.object_with_io.wait(time_to_wait)
            else:
                print "Too slow"

    def loop_core(self):
        self.callback(self.object_with_io)

    def execute(self):
        self._running.set()

    def is_executed(self):
        return self._running.is_set()

    def stop(self):
        self._running.clear()

    def _terminate(self):
        self._to_terminate.set()