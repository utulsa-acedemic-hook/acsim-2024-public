"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
"""
import copy
import numpy as np
import parameters.control_parameters as AP

# from tools.transfer_function import TransferFunction
from tools.wrap import wrap
from controllers.pi_control import PIControl
from controllers.pid_control import PIDControl
from controllers.pd_control_with_rate import PDControlWithRate
from controllers.tf_control import TFControl
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta

alpha_elev_kp = 0.0
alpha_elev_ki = -0.1
alpha_elev_kd = 0.0
beta_rud_kp = 0.0
beta_rud_ki = 0.0
beta_rud_kd = 0.0
va_throttle_kp = 0.0
va_throttle_ki = 0.0001
va_throttle_kd = 0.0

class Autopilot:
    def __init__(self, ts_control, mav, delta):
        self.delta_trim = delta
        self.init_state = copy.deepcopy(mav.true_state)
        # instantiate lateral-directional controllers
        self.elevator_from_alpha = PIDControl(
            kp=alpha_elev_kp,
            ki=alpha_elev_ki,
            kd=alpha_elev_kd,
            limit=np.radians(45),
            # initial_integrator=self.delta_trim.elevator / alpha_elev_ki,
        )
        self.airspeed_from_throttle = PIControl(
            kp=va_throttle_kp,
            ki=va_throttle_ki,
            Ts=ts_control,
            limit=1.0,
            initial_integrator=self.delta_trim.throttle / va_throttle_ki,
        )
        # self.roll_from_aileron = PDControlWithRate(
        #     kp=AP.roll_kp, kd=AP.roll_kd, limit=np.radians(45)
        # )
        # self.course_from_roll = PIControl(
        #     kp=AP.course_kp, ki=AP.course_ki, Ts=ts_control, limit=np.radians(30)
        # )
        # self.yaw_damper = TransferFunction(
        #                 num=np.array([[AP.yaw_damper_kr, 0]]),
        #                 den=np.array([[1, AP.yaw_damper_p_wo]]),
        #                 Ts=ts_control)
        self.yaw_damper = TFControl(
            k=AP.yaw_damper_kr,
            n0=0.0,
            n1=1.0,
            d0=AP.yaw_damper_p_wo,
            d1=1,
            Ts=ts_control,
        )

        # instantiate longitudinal controllers
        self.pitch_from_elevator = PIDControl(
            kp=AP.pitch_kp,
            ki=0.001,
            kd=AP.pitch_kd,
            limit=np.radians(45),
            initial_integrator=self.delta_trim.elevator / 0.001,
        )
        self.altitude_from_pitch = PIControl(
            kp=AP.altitude_kp, ki=AP.altitude_ki, Ts=ts_control, limit=np.radians(30)
        )
        if AP.airspeed_throttle_ki == 0:
            AP.airspeed_throttle_ki = 0.001
        self.airspeed_from_throttle = PIControl(
            kp=AP.airspeed_throttle_kp,
            ki=AP.airspeed_throttle_ki,
            Ts=ts_control,
            limit=1.0,
            initial_integrator=self.delta_trim.throttle / AP.airspeed_throttle_ki,
        )
        self.commanded_state = MsgState()

    def update(self, cmd, state):

        #### TODO #####
        delta = MsgDelta(elevator=0, aileron=0, rudder=0, throttle=0)
        # lateral autopilot

        # longitudinal autopilot
        delta.elevator = self.elevator_from_alpha.update(self.init_state.alpha, state.alpha)
        delta.throttle = self.airspeed_from_throttle.update(0, 0)

        # construct control outputs and commanded states

        self.commanded_state.altitude = 0
        self.commanded_state.Va = 0
        self.commanded_state.phi = 0
        self.commanded_state.theta = 0
        self.commanded_state.chi = 0
        return delta, self.commanded_state

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output
