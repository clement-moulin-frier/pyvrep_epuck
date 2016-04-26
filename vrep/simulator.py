from . import vrep
from robots.epuck import Epuck
from time import sleep
from vrepConst import *


class Simulator(object):
    def __init__(self):
        vrep.simxFinish(-1) # just in case, close all opened connections
        self._clientID = vrep.simxStart('127.0.0.1',19997, True, True, 5000, 5) # Connect to V-REP        
        self.robots = []

    def start(self):
        vrep.simxStartSimulation(self._clientID, vrep.simx_opmode_oneshot_wait);
        sleep(1)

    def stop(self):
        vrep.simxStopSimulation(self._clientID, vrep.simx_opmode_oneshot_wait)

    def get_epuck(self, suffix=""):
        self.robots.append(Epuck(self._clientID, suffix))
        return self.robots[-1]
    def load_scene(self, file_name):
        vrep.simxLoadScene(self._clientID, file_name, 0, simx_opmode_oneshot_wait)
