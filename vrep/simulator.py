

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

from ..routine import Routine, RoutineManager
from ..logger import Logger
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
    if len(epucks) == 0:
        epucks = simulator.robots
    simulator.close()
    for e in epucks:
        e.close()
        del e
    del simulator
    sleep(0.1)
    close_all_connections()


def sphere_apparition(simulator, min_pos, max_pos):
    if not hasattr(simulator, "n_spheres"):
        simulator.n_spheres = 0
    if not hasattr(simulator, "eatable_objects"):
        simulator.eatable_objects = []       
    min_pos = array(min_pos)
    max_pos = array(max_pos)
    name = "Sphere_" + str(simulator.n_spheres + 1)
    pos = rand(3) * (max_pos - min_pos) + min_pos
    simulator.io.add_sphere(name, pos, [0.1, 0.1, 0.1], 0.5)
    simulator.eatable_objects.append(name)
    simulator.io._inject_lua_code("simSetObjectSpecialProperty({}, {})".format(simulator.io.get_object_handle(name), vrep.sim_objectspecialproperty_detectable_all))
    simulator.n_spheres += 1
    for robot in simulator.robots:
        robot.register_object(name)

def eating(simulator):
    if not hasattr(simulator, "eatable_objects"):
        simulator.eatable_objects = []  
        for i_r, r in enumerate(simulator.robots):
            simulator.subscribe(topic=(i_r, "eat"), subscriber=r)              
    objects_to_remove = []
    for i_r, robot in enumerate(simulator.robots):
        robot_pos = array(robot.position())
        for obj in simulator.eatable_objects:
            try:
                obj_pos = array(simulator.get_object_position(obj))
            except VrepIOErrors:
                break
            if obj not in objects_to_remove and norm(robot_pos - obj_pos) < 0.1:
                objects_to_remove.append(obj)
                simulator.emit((i_r, "eat"), simulator.io.get_simulation_current_time())
    for obj in objects_to_remove:
        simulator.eatable_objects.remove(obj)
        simulator.remove_object(obj)

class Simulator(Observable):
    def __init__(self, vrep_host='127.0.0.1', vrep_port=19997, scene=None, start=False):
        self.io = VrepIO(vrep_host, vrep_port, scene, start)
        # vrep.simxFinish(-1) # just in case, close all opened connections
        # self._clientID = vrep.simxStart('127.0.0.1',19997, True, True, 5000, 5) # Connect to V-REP
        self.robots = []

        self.vrep_mode = vrep_mode

        self.n_robots = 0

        self.suffix_to_epuck = {}
        self.seconds_to_run = 0

        self._condition = threading.Condition()
        self.routine_manager = RoutineManager()

        self.eatable_objects = []

        self.logger = Logger()

        Observable.__init__(self)

    def wait(self, seconds):
        start = self.io.get_simulation_current_time()
        while self.io.get_simulation_current_time() - start < seconds:
            sleep(0.005)


    def close(self):
        self.detach_all_routines()
        sleep(0.3)
        self.io.stop_simulation()
        self.io.close()

    def get_object_position(self, object_name):
        return array(self.io.get_object_position(object_name))

    def set_object_position(self, object_name, position):
        return self.io.set_object_position(object_name, position)

    def add_sphere(self, position, sizes=[0.1, 0.1, 0.1], mass=0.5, eatable=True):
        name = "Sphere_" + str(len(self.eatable_objects) + 1)
        self.io.add_sphere(name, position, sizes, mass)
        if eatable:
            self.eatable_objects.append(name)
        self.io._inject_lua_code("simSetObjectSpecialProperty({}, {})".format(self.io.get_object_handle(name), vrep.sim_objectspecialproperty_detectable_all))
        self.n_spheres = len(self.eatable_objects)
        for robot in self.robots:
            robot.register_object(name)    

    def start_sphere_apparition(self, period=5., min_pos=[-1., -1., .1], max_pos=[1., 1., 1.]):
        self.attach_routine(sphere_apparition, freq=1./period, min_pos=min_pos, max_pos=max_pos)
        self.attach_routine(eating, freq=4.)
        self.start_routine(sphere_apparition)
        self.start_routine(eating)

    def stop_sphere_apparition(self):
        self.stop_routine(sphere_apparition)
        self.stop_routine(eating)

    def add_log(self, topic, data):
        self.logger.add(topic, data)

    def get_log(self, topic):
        return self.logger.get_log(topic)

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

    def get_epuck(self, use_proximeters=list(range(8)), verbose=False):
        suffix = self._vrep_epuck_suffix(self.n_robots)
        epuck = Epuck(pypot_io=VrepIO(self.io.vrep_host, self.io.vrep_port + self.n_robots + 1), simulator=self, robot_id=self.n_robots, use_proximeters=use_proximeters, suffix=suffix)
        self.suffix_to_epuck[suffix] = epuck
        self.robots.append(epuck)
        self.n_robots += 1
        if verbose: print(self.robots[-1])
        return self.robots[-1]

    def get_epuck_list(self, n_epucks, verbose=False):
        return [self.get_epuck(verbose=verbose) for _ in range(n_epucks)]

    def remove_object(self, name):
        self.io.call_remote_api("simxRemoveObject", self.io.get_object_handle(name), sending=True)


    def attach_routine(self,callback, freq, **kwargs):
        routine = Routine(self, callback, self._condition, freq, **kwargs)
        self.routine_manager.attach(routine)

    def detach_routine(self, callback):
        self.routine_manager.detach(callback)
        print("Routine " + callback.__name__ + " detached")


    def detach_all_routines(self):
        self.routine_manager.detach_all()

    def start_routine(self, callback):
        if self.routine_manager.start(callback):
            print("Routine " + callback.__name__ + " started")

    def start_all_routines(self):
        self.routine_manager.start_all()

    def stop_routine(self, callback):
        if self.routine_manager.stop(callback):
            print("Routine " + callback.__name__ + " stopped")

    def stop_all_routines(self):
        self.routine_manager.stop_all()

    def check_routines(self):
        self.routine_manager.check()
