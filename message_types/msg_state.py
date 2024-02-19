"""
msgState 
    - messages type for state, that will be passed between blocks in the architecture
    
part of mavPySim 
    - Beard & McLain, PUP, 2012
    - Update history:  
        1/9/2019 - RWB
        3/30/2022 - RWB
"""

import numpy as np


class MsgState:
    def __init__(self):
        self.north = 0.0  # inertial north position in meters
        self.east = 0.0  # inertial east position in meters
        self.altitude = 100.0  # inertial altitude in meters
        self.phi = 0.0  # roll angle in radians
        self.theta = 0.0  # pitch angle in radians
        self.psi = 0.0  # yaw angle in radians
        self.Va = 25.0  # airspeed in meters/sec
        self.alpha = 0.0  # angle of attack in radians
        self.beta = 0.0  # sideslip angle in radians
        self.p = 0.0  # roll rate in radians/sec
        self.q = 0.0  # pitch rate in radians/sec
        self.r = 0.0  # yaw rate in radians/sec
        self.Vg = 25.0  # groundspeed in meters/sec
        self.gamma = 0.0  # flight path angle in radians
        self.chi = 0.0  # course angle in radians
        self.wn = 0.0  # inertial windspeed in north direction in meters/sec
        self.we = 0.0  # inertial windspeed in east direction in meters/sec
        self.bx = 0.0  # gyro bias along roll axis in radians/sec
        self.by = 0.0  # gyro bias along pitch axis in radians/sec
        self.bz = 0.0  # gyro bias along yaw axis in radians/sec
        self.camera_az = 0.0  # camera azimuth angle
        self.camera_el = np.radians(-90)  # camera elevation angle
        self.u = 0.0
        self.v = 0.0
        self.w = 0.0
