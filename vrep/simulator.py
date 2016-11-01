from __future__ import division

from ..robots.epuck import Epuck

from pypot.vrep.io import VrepIO, VrepIOErrors
from pypot.vrep.remoteApiBindings import vrep
from pypot.vrep import close_all_connections
from pypot.vrep.io import vrep_mode

from time import sleep
import threading

from numpy.linalg import norm
from numpy import array
from numpy.random import rand
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
    sleep(0.1)
    close_all_connections()
    sleep(0.1)
    close_all_connections()
    sleep(0.1)
    simulator = Simulator()
    simulator.io.restart_simulation()
    epucks = [simulator.get_epuck(use_proximeters=use_proximeters) for _ in range(n_epucks)]
    if n_epucks == 1:
        return simulator, epucks[0]
    else:
        return [simulator] + epucks

def close_session(simulator, *epucks):
    simulator.close()
    for e in epucks:
        e.close()
        del e
    del simulator
    sleep(0.1)
    close_all_connections()



class Simulator(Observable):
    def __init__(self, vrep_host='127.0.0.1', vrep_port=19997, scene=None, start=False):
        self.io = VrepIO(vrep_host, vrep_port, scene, start)
        # vrep.simxFinish(-1) # just in case, close all opened connections
        # self._clientID = vrep.simxStart('127.0.0.1',19997, True, True, 5000, 5) # Connect to V-REP
        self.robots = []

        self.vrep_mode = vrep_mode

        self.t = 0.
        self.dt = 100.

        self.sphere_apparition_period = 0

        self.n_robots = 0

        self.suffix_to_epuck = {}

        self._thread = threading.Thread(target=lambda: self._run(0))
        self._running = threading.Event()
        self._running.clear()

        Observable.__init__(self)

    def run(self):
        self._running.set()
        self._thread.start()

    def wait(self):
        self._thread.join()

    def stop(self):
        self._running.clear()

    def close(self):
        self.stop()
        while self._thread.isAlive():
            sleep(0.1)
        self.io.stop_simulation()
        self.io.close()

    def start_sphere_apparition(self, period=5., min_pos=[-1., -1., .1], max_pos=[1., 1., 1.]):
        for i_r, r in enumerate(self.robots):
            self.subscribe(topic=(i_r, "eat"), subscriber=r)
        self.sphere_min_pos = array(min_pos)
        self.sphere_max_pos = array(max_pos)
        self.sphere_apparition_period = period
        if not self._thread.isAlive():
            self.run()

    def stop_sphere_apparition(self):
        self.sphere_apparition_period = 0

    def _vrep_epuck_suffix(self, num):
        if num == 0:
            return ""
        else:
            return "#" + str(num - 1)

    def epuck_from_object_name(self, name):
        if not name.startswith("ePuck"):
            return None
        elif "#" in name:
            suffix = "#" + name.split("#")[1]
        else:
            suffix = ""
        return self.suffix_to_epuck[suffix]

    def get_epuck(self, use_proximeters=range(8), verbose=False):
        suffix = self._vrep_epuck_suffix(self.n_robots)
        epuck = Epuck(pypot_io=VrepIO(self.io.vrep_host, self.io.vrep_port + self.n_robots + 1), simulator=self, use_proximeters=use_proximeters, suffix=suffix)
        self.suffix_to_epuck[suffix] = epuck
        self.robots.append(epuck)
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
        while not seconds or self.t < seconds * 1000.:
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
                        self.emit((i_r, "eat"), self.io.get_simulation_current_time())
            for obj in objects_to_remove:
                self.object_names.remove(obj)
                self.remove_object(obj)

            if self.sphere_apparition_period and start_time > last_sphere_t + self.sphere_apparition_period:
                name = "Sphere_" + str(n_objects + 1)
                pos = rand(3) * (self.sphere_max_pos - self.sphere_min_pos) + self.sphere_min_pos
                self.io.add_sphere(name, pos, [0.1, 0.1, 0.1], 0.5)
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
