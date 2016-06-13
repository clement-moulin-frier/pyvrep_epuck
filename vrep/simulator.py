from pypot.vrep.io import VrepIO, VrepIOErrors
from ..robots.epuck import Epuck
from pypot.vrep.remoteApiBindings import vrep
from pypot.vrep.remoteApiBindings.vrepConst import simx_opmode_oneshot_wait

from random import  shuffle
from time import time, sleep
import threading
from numpy.random import rand
from numpy.linalg import norm
from numpy import array

from pypot.vrep.io import vrep_mode


class Simulator(VrepIO):
    def __init__(self, vrep_host='127.0.0.1', vrep_port=19997, scene=None, start=False):
        VrepIO.__init__(self, vrep_host, vrep_port, scene, start)
        # vrep.simxFinish(-1) # just in case, close all opened connections
        # self._clientID = vrep.simxStart('127.0.0.1',19997, True, True, 5000, 5) # Connect to V-REP        
        self.robots = []

        self.vrep_mode = vrep_mode

        self.t = 0.
        self.dt = 10.

        self._running = threading.Event()

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

    # def start(self):
    #     vrep.simxStartSimulation(self._clientID, vrep.simx_opmode_oneshot_wait);
    #     sleep(1)

    # def stop(self):
    #     vrep.simxStopSimulation(self._clientID, vrep.simx_opmode_oneshot_wait)

    def get_epuck(self, suffix=""):
        self.robots.append(Epuck(self.client_id, pypot_io=self, suffix=suffix))
        return self.robots[-1]

    # def load_scene(self, file_name):
    #     vrep.simxLoadScene(self._clientID, file_name, 0, simx_opmode_oneshot_wait)

    # def get_object_handle(self, name):
    #     _, obj_handle = vrep.simxGetObjectHandle(self._clientID, name, vrep.simx_opmode_oneshot_wait)
    #     return obj_handle

    def remove_object(self, name):
        vrep.simxRemoveObject(self.client_id, self.get_object_handle(name), vrep.simx_opmode_oneshot)

    def _run(self, seconds):
        n_objects = 0
        self.object_names = []
        while self.t < seconds * 1000.:
            start_time = time()
            
            # print "1"
            # objects_to_remove = []
            # for robot in self.robots:
            #     robot_pos = array(robot.position())
            #     for obj in self.object_names:
            #         try:
            #             obj_pos = array(self.get_object_position(obj))
            #         except VrepIOErrors:
            #             break
            #         if obj not in objects_to_remove and norm(robot_pos - obj_pos) < 0.1:
            #             objects_to_remove.append(obj)
            # for obj in objects_to_remove:
            #     self.object_names.remove(obj)
            #     self.remove_object(obj)
                    
            # print "2"
            if rand() < 0.02:
                name = "Sphere_" + str(n_objects + 1)
                self.add_sphere(name, [0.0, 0.0, 0.2], [0.1, 0.1, 0.1], 0.5)
                self.object_names.append(name)
                self._inject_lua_code("simSetObjectSpecialProperty({}, {})".format(self.get_object_handle(name), vrep.sim_objectspecialproperty_detectable_all))
                n_objects += 1
                for robot in self.robots:
                    robot.register_object(name)
                
            # print "3"
            while time() < start_time + self.dt / 1000.:
                # print "wait"
                sleep(self.dt/(100. * 1000))
            
            self.t += self.dt

            if not self._running.is_set():
                break

        self._running.clear()            


