from models.mav_dynamics_control import MavDynamics
from message_types.msg_delta import MsgDelta
import numpy as np
from scipy.optimize import minimize

def compute_trim(mav: MavDynamics, delta: MsgDelta) -> MsgDelta:
    x0 = [delta.elevator, delta.throttle, mav._alpha]
    bounds = [(-1, 1), (0, 1), (0, np.deg2rad(12))]
    res = minimize(mav.compute_trim_obj_func, x0, bounds=bounds, tol=1e-8, method='L-BFGS-B')
    mav.initialize_velocity(mav._Va, res.x[2], mav._beta)
    mav.initialize_state()
    delta.elevator = res.x[0]
    delta.throttle = res.x[1]
    print(f"results: {res.x[0]}, {res.x[1]}, {res.x[2]}")
    return delta
