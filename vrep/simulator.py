from ..robots.epuck import Epuck

from pypot.vrep.io import VrepIO, VrepIOErrors
from pypot.vrep.remoteApiBindings import vrep
from pypot.vrep import close_all_connections
from pypot.vrep.io import vrep_mode

from time import sleep
import threading

from numpy.linalg import norm
from numpy import array

from .observer import Observable


def close():
    close_all_connections()

def get_session(n_epucks=1, use_proximeters=[2, 3], old_simulator=None, old_epuck=None):
    if old_simulator is not None:
        old_simulator.stop()
        old_simulator.io.close()
        del old_simulator
    if old_epuck is not None:
        old_epuck.stop()
        old_epuck.io.close()
        del old_epuck
    close_all_connections()
    simulator = Simulator()
    simulator.io.restart_simulation()
    epucks = [simulator.get_epuck(use_proximeters) for _ in range(n_epucks)]
    if n_epucks == 1:
        return simulator, epucks[0]
    else:
        return [simulator] + epucks


class Simulator(Observable):
    def __init__(self, vrep_host='127.0.0.1', vrep_port=19997, scene=None, start=False, sphere_apparition_period=5.):
        self.io = VrepIO(vrep_host, vrep_port, scene, start)
        # vrep.simxFinish(-1) # just in case, close all opened connections
        # self._clientID = vrep.simxStart('127.0.0.1',19997, True, True, 5000, 5) # Connect to V-REP
        self.robots = []

        self.vrep_mode = vrep_mode

        self.t = 0.
        self.dt = 100.

        self.sphere_apparition_period = sphere_apparition_period

        self.n_robots = 0

        self._running = threading.Event()

        Observable.__init__(self)

    def run(self, seconds):
        self._running.set()
        self._thread = threading.Thread(target=lambda: self._run(seconds))
        self._thread.start()

    def wait(self):
        """ Wait for the end of the run of the experiment. """
        self._thread.join()

    def stop(self):
        """ Stop the experiment. """
        self._running.clear()

    def close_connection(self):
        close_all_connections()

    def _vrep_epuck_suffix(self, num):
        if num == 0:
            return ""
        else:
            return "#" + str(num - 1)

    def get_epuck(self, use_proximeters=range(8), verbose=False):
        self.robots.append(Epuck(pypot_io=VrepIO(self.io.vrep_host, self.io.vrep_port + self.n_robots + 1), use_proximeters=use_proximeters, suffix=self._vrep_epuck_suffix(self.n_robots)))
        self.n_robots += 1
        if verbose: print self.robots[-1]
        return self.robots[-1]

    def get_epuck_list(self, n_epucks, verbose=False):
        return [self.get_epuck(verbose) for _ in range(n_epucks)]

    def remove_object(self, name):
        self.io.call_remote_api("simxRemoveObject", self.io.get_object_handle(name), sending=True)

    def _run(self, seconds):

        self._running.set()

        n_objects = 0
        self.object_names = []
        last_sphere_t = self.io.get_simulation_current_time()
        while self.t < seconds * 1000.:
            start_time = self.io.get_simulation_current_time()

            objects_to_remove = []
            for i_r, robot in enumerate(self.robots):
                robot_pos = array(robot.position())
                for obj in self.object_names:
                    try:
                        obj_pos = array(self.io.get_object_position(obj))
                    except VrepIOErrors:
                        break
                    if obj not in objects_to_remove and norm(robot_pos - obj_pos) < 0.1:
                        objects_to_remove.append(obj)
                        self.emit((i_r, "eat"), True)
            for obj in objects_to_remove:
                self.object_names.remove(obj)
                self.remove_object(obj)

            if start_time > last_sphere_t + self.sphere_apparition_period:
                name = "Sphere_" + str(n_objects + 1)
                self.io.add_sphere(name, [-1., -1., 0.2], [0.1, 0.1, 0.1], 0.5)
                self.object_names.append(name)
                self.io._inject_lua_code("simSetObjectSpecialProperty({}, {})".format(self.io.get_object_handle(name), vrep.sim_objectspecialproperty_detectable_all))
                n_objects += 1
                for robot in self.robots:
                    robot.register_object(name)
                last_sphere_t = start_time

            while self.io.get_simulation_current_time() < start_time + self.dt / 1000.:
                # print "wait"
                sleep(self.dt / (100. * 1000))

            self.t += self.dt / 1000.

            if not self._running.is_set():
                break

        self._running.clear()
