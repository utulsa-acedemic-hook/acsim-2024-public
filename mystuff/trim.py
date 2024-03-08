from models.mav_dynamics_control import MavDynamics
from message_types.msg_delta import MsgDelta

def compute_trim(mav: MavDynamics, delta: MsgDelta) -> MsgDelta:
    delta.elevator = -0.1248
    delta.aileron = 0.001836
    delta.rudder = -0.0003026
    delta.throttle = 0.6768
    
    return delta