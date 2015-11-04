from vrep import vrep
from math import sqrt
from numpy import mean, array, argmax
from time import sleep
from threading import Event
from threading import Thread as ParralelClass
# from multiprocessing import Process as ParralelClass

class Epuck(object):
    def __init__(self):
        vrep.simxFinish(-1) # just in case, close all opened connections
        self._clientID = vrep.simxStart('127.0.0.1',19997, True, True, 5000, 5) # Connect to V-REP
        _, self._left_joint = vrep.simxGetObjectHandle(self._clientID, 'ePuck_leftJoint', vrep.simx_opmode_oneshot_wait)
        _, self._right_joint = vrep.simxGetObjectHandle(self._clientID, 'ePuck_rightJoint', vrep.simx_opmode_oneshot_wait)
        _, self._light_sensor = vrep.simxGetObjectHandle(self._clientID, 'ePuck_lightSensor', vrep.simx_opmode_oneshot_wait)
        _, self._camera = vrep.simxGetObjectHandle(self._clientID, 'ePuck_camera', vrep.simx_opmode_oneshot_wait)
        
        self._prox_handles = []
        for i in range(1,9):
            _, p = vrep.simxGetObjectHandle(self._clientID, 'ePuck_proxSensor' + str(i), vrep.simx_opmode_oneshot_wait)
            self._prox_handles.append(p)
        
        # First calls with simx_opmode_streaming
        for i in range(8):
            vrep.simxReadProximitySensor(self._clientID, self._prox_handles[i], vrep.simx_opmode_streaming)
        _, self.camera_resolution, _ = vrep.simxGetVisionSensorImage(self._clientID, self._camera, options=0, operationMode=vrep.simx_opmode_streaming)
        _, self._light_sensor_resolution, _ = vrep.simxGetVisionSensorImage(self._clientID, self._light_sensor, options=0, operationMode=vrep.simx_opmode_streaming)
        
        self._body = vrep.simxGetObjectHandle(self._clientID, "ePuck_bodyElements", vrep.simx_opmode_oneshot_wait)
        self.wheel_diameter = 4.25 * 10 ** -2
        self.base_lenght = 7 * 10 ** -2
        
        self._prox_aliases = {"all" : range(8),
                             "all-but-rear" : range(6),
                             "front" : [2, 3],
                             "rear" : [6, 7],
                             "front-left" : [0, 1, 2],
                             "front-right": [3, 4, 5]}
        
        self.avoid_fwd_spd = 0.1 
        self.avoid_rot_spd = 1.
        self.fwd_spd = 0.2

        vrep.simxGetFloatSignal(self._clientID, "CurrentTime", vrep.simx_opmode_streaming)

        self.freq = 100
        self._behaviors = {}
        self._runnings = {}
        self._to_detach = {}

        # self._running = threading.Event()
        # self._running.set()
        
    def start(self):
        self.left_vel(0.)
        self.right_vel(0.)
        vrep.simxStartSimulation(self._clientID, vrep.simx_opmode_oneshot_wait);
    
    def stop(self):
        vrep.simxStopSimulation(self._clientID, vrep.simx_opmode_oneshot_wait)

    def simulation_time(self):
        _, sim_time = vrep.simxGetFloatSignal(self._clientID, "CurrentTime", vrep.simx_opmode_buffer)
        return sim_time

    def wait(self, sec):
        sim_time = self.simulation_time()
        while self.simulation_time() - sim_time < sec:
            pass
            sleep(0.001)
        
    def left_vel(self, vel):
        vrep.simxSetJointTargetVelocity(self._clientID, self._left_joint, vel, vrep.simx_opmode_oneshot)
        
    def right_vel(self, vel):
        vrep.simxSetJointTargetVelocity(self._clientID, self._right_joint, vel, vrep.simx_opmode_oneshot)
    
    def move(self, forward, rotate=0.):
        v_l = ( (2.0 * forward) - (rotate * self.base_lenght) ) / (self.wheel_diameter)
        v_r = ( (2.0 * forward) + (rotate * self.base_lenght) ) / (self.wheel_diameter)
        vrep.simxPauseCommunication(self._clientID, True)
        self.left_vel(v_l)
        self.right_vel(v_r)
        vrep.simxPauseCommunication(self._clientID, False)
    
    def proximeters(self, group="all"):
        distances = []
        vrep.simxPauseCommunication(self._clientID, True)
        for i in range(8):
            res, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(self._clientID, self._prox_handles[i], vrep.simx_opmode_buffer)
            if detectionState:
                distances.append(1000 * sqrt(sum([x ** 2 for x in detectedPoint])))
            else:
                distances.append(0)
        vrep.simxPauseCommunication(self._clientID, False)
        return array(distances)[self._prox_aliases[group]]

    def is_min_distance(self, group, min_dist):
        proxs = self.proximeters(group=group)
        proxs = proxs[proxs != 0]
        if len(proxs):
            return any(proxs < min_dist)
        else:
            return False
    
    def camera_image(self):
        _, resolution, image = vrep.simxGetVisionSensorImage(epuck._clientID, self._camera, options=0, operationMode=vrep.simx_opmode_buffer)
        image.resize(resolution[0], resolution[1], 3)
        return image
    
    def light_sensor_image(self):
        _, resolution, image = vrep.simxGetVisionSensorImage(epuck._clientID, self._light_sensor, options=0, operationMode=vrep.simx_opmode_buffer)
        image.resize(resolution[0], resolution[1], 3)
        return image   

    def attach_behavior(self, callback, freq=None):
        if freq is None:
            period = 1. / self.freq
        else:
            period = 1. / freq
        self._runnings[callback.__name__] = Event()
        self._to_detach[callback.__name__] = Event()
        self._to_detach[callback.__name__].clear()
        def loop(robot):
            while True: 
                if self._runnings[callback.__name__].is_set():
                    callback(robot)
                robot.wait(period)     
                if self._to_detach[callback.__name__].is_set():
                    break
            self._to_detach[callback.__name__].clear()
        self._behaviors[callback.__name__] = ParralelClass(target=loop, args=(self,))
        self._runnings[callback.__name__].clear()
        self._behaviors[callback.__name__].start()

    def detach_behavior(self, callback_name):
        if not callback_name in self._behaviors:
            print("Warning: " + callback_name + " was not attached")
        else:
            self._to_detach[callback_name].set()
            del self._behaviors[callback_name]

    def start_behavior(self, callback_name):
        if not callback_name in self._behaviors:
            print("Warning: " + callback_name + " is not attached")
        else:
            self._runnings[callback_name].set()  
            self._to_detach[callback_name].clear() 
            

    def stop_behavior(self, callback_name):
        if not callback_name in self._behaviors:
            print("Warning: " + callback_name + " is not attached")
        else:
            self._runnings[callback_name].clear()              