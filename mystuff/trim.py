from models.mav_dynamics_control import MavDynamics
from message_types.msg_delta import MsgDelta
import numpy as np

def compute_trim(mav: MavDynamics, delta: MsgDelta) -> MsgDelta:
    delta.elevator = 0
    delta.aileron = 0
    delta.rudder = 0
    delta.throttle = 0
    result = np.linalg.norm(mav._forces_moments(delta=delta))
    print(f"result: {result}")
    return delta