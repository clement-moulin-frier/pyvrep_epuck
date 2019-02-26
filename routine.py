from threading import Thread as ParralelClass
from threading import Event, Condition
from time import sleep

class RoutineManager(object):
    def __init__(self):
        self._routines = {}

    def attach(self, routine):
        self._routines[routine.callback] = routine
        self._routines[routine.callback].start()

    # def attach(self, class_to_create, callback, freq):
    #     self._routines[callback] = class_to_create(self, callback, self.condition, freq)
    #     self._routines[callback].start()

    def detach(self, callback):
        if callback not in self._routines:
            print(("Warning: " + callback.__name__ + " was not attached"))
        else:
            self._routines[callback].stop()  # just in case
            self._routines[callback]._terminate()
            sleep(self._routines[callback].period)
            del self._routines[callback]        

    def detach_all(self):
        dict_copy = dict(self._routines)  # because one can't modify the dict during the loop on itself
        for callback, item in dict_copy.items():
            self.detach(callback)

    def start(self, callback):
        if callback not in self._routines:
            print(("Warning: " + callback.__name__ + " is not attached"))
            return False
        else:
            self._routines[callback].execute()
            return True

    def start_all(self):
        for callback in self._routines:
            self.start(callback)       

    def stop(self, callback):
        if callback not in self._routines:
            print(("Warning: " + callback.__name__ + " is not attached"))
            return False
        else:
            self._routines[callback].stop()
            return True

    def stop_all(self):
        for callback in self._routines:
            self.stop(callback)         

    def check(self, label):
        if not len(self._routines):
            print("No " + label.lower() + " attached")
        for callback, obj in self._routines.items():
            print(label + " \"{name}\" is attached and {started}".format(name=callback.__name__, started="STARTED" if obj._running.is_set() else "NOT STARTED."))



class Routine(ParralelClass):

    def __init__(self, object_with_io, callback, condition, freq, **callback_kwargs):
        ParralelClass.__init__(self)
        self.period = 1. / freq
        self.object_with_io = object_with_io
        self.callback = callback
        self.callback_kwargs = callback_kwargs

        self.condition = condition
        self._running = Event()
        self._running.clear()
        self._to_terminate = Event()
        self._to_terminate.clear()
        self.verbose = False

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
            elif self.verbose:
                print("Too slow")

    def loop_core(self):
        self.callback(self.object_with_io, **self.callback_kwargs)

    def execute(self):
        self._running.set()

    def is_executed(self):
        return self._running.is_set()

    def stop(self):
        self._running.clear()

    def _terminate(self):
        self._to_terminate.set()